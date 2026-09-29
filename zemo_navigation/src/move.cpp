#include "zemo_navigation/move.hpp"

#define sign(x) (((x) > 0) - ((x) < 0))

namespace zemo_navigation
{
  static std::string debugMessage(MoveCommand::Result::SharedPtr result)
  {
    switch (result->error)
    {
      case MoveDebug::ERROR_TIMEOUT:
        return "ERROR_TIMEOUT, allowed time exceeded for the Move action!";
      case MoveDebug::ERROR_STUCK:
        return "";
      case MoveDebug::ERROR_OBSTACLE:
        return "ERROR_OBSTACLE, collision detected!";
      case MoveDebug::ERROR_MISSING_TRANSFORM:
        return "ERROR_MISSING_TRANSFORM, current robot pose is not available, or a Transform!";
      case MoveDebug::ERROR_CANCELED:
        return "ERROR_CANCELED, move canceld!";
      case MoveDebug::ERROR_MODE:
        return "ERROR_MODE, invalide mode value!";
      default:
        return "";
    }
  }


  rclcpp_action::GoalResponse MoveActionServer::handle_goal(const rclcpp_action::GoalUUID &uuid , std::shared_ptr<const MoveCommand::Goal> goal)
  {
    (void)uuid;
    // todo active goal problem
    if (current_goal_ != nullptr && current_goal_->is_active())
    {
      RCLCPP_WARN(get_logger(), "Goal rejected, one goal already active!");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // samo prvi put hocemo da inicjalizujemo ovo, obratiti paznju mozda ne stigne da publish za prvi
    if (current_goal_ == nullptr)
    {
      nav2_init();
    }
    
    // init command
    command_->header = goal->header;
    command_->target = goal->target;
    command_->linear_properties = goal->linear_properties;
    command_->angular_properties = goal->angular_properties;
    command_->ignore_obstacle = goal->ignore_obstacle;
    command_->enalble_sensor = goal->enalble_sensor;
    command_->timeout = goal->timeout;
    command_->mode = goal->mode;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse MoveActionServer::handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Rceived a cancle requeste");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void MoveActionServer::handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    current_goal_ = goal_handle;
    auto execute_thread = [this, goal_handle](){ return this->execute(goal_handle);};
    std::thread{execute_thread}.detach();
  }

  void MoveActionServer::execute(const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    // loop time init, mozda moze i looprate viid pa probaj todo
    rclcpp::Duration period_duration(std::chrono::nanoseconds(1000000000 / update_rate_));
    rclcpp::Time period_end_time = now();

    if(!init_move())
    {
      goal_handle->abort(result_);
      RCLCPP_WARN(get_logger(), "Init failed");
      //goal_active_ = false;
      return;
    }

    while (rclcpp::ok())
    {
      period_end_time += period_duration;
      auto sleep_time = std::chrono::nanoseconds((period_end_time - this->now()).nanoseconds());
      if (sleep_time > std::chrono::nanoseconds::zero())
      {
        std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_time));
      }

      if(goal_handle->is_canceling())
      {
        if(result_->error == MoveDebug::ERROR_NONE)
          result_->error = MoveDebug::ERROR_CANCELED;
        state_ = STATE_STOPPING;
      }

      update();

      if (finished_)
      {
        if (result_->error == MoveDebug::ERROR_NONE)
        {
          goal_handle->succeed(result_);
        }
        else if(result_->error == MoveDebug::ERROR_CANCELED)
        {
          goal_handle->canceled(result_);
          RCLCPP_WARN(get_logger(), "%s", debugMessage(result_).c_str());
        }
        else
        {
          goal_handle->abort(result_);
          RCLCPP_WARN(get_logger(), "%s", debugMessage(result_).c_str());
        }
        return;
      }
    }

    goal_handle->abort(result_);
    RCLCPP_WARN(get_logger(), "%s", debugMessage(result_).c_str());

  }

  bool MoveActionServer::update_odom_target_tf()
  {
    tf2::Transform tf_global_target;
    tf_global_target.setOrigin(tf2::Vector3(command_->target.x, command_->target.y, 0.0));
    tf_global_target.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), command_->target.theta));

    geometry_msgs::msg::TransformStamped tf_global_odom_message;
    try
    {
      tf_global_odom_message = tf_->lookupTransform(command_->header.frame_id, odom_frame_, command_->header.stamp);
    }
    catch (const tf2::TransformException &ex)
    {
      result_->error = MoveDebug::ERROR_MISSING_TRANSFORM;
      //state_msg_.error = mep3_msgs::msg::MoveState::ERROR_MISSING_TRANSFORM;
      //state_pub_->publish(state_msg_);

      return false;
    }

    tf2::Transform tf_global_odom;
    tf2::convert(tf_global_odom_message.transform, tf_global_odom);
    tf_odom_target_ = tf_global_odom.inverse() * tf_global_target;
    //target_updated_ = true;   todo

    return true;
  }

  bool MoveActionServer::init_move()
  {
    // Apply defaults
    if (command_->header.frame_id == "")
      command_->header.frame_id = "map";
    if (command_->timeout.sec <= 0 && command_->timeout.nanosec <= 0)
      command_->timeout = command_timeout_;
    if (command_->linear_properties.max_velocity == 0.0)
      command_->linear_properties.max_velocity = default_command_->linear_properties.max_velocity;
    if (command_->linear_properties.max_acceleration == 0.0)
      command_->linear_properties.max_acceleration = default_command_->linear_properties.max_acceleration;
    if (command_->linear_properties.max_jerk == 0.0)
      command_->linear_properties.max_jerk = default_command_->linear_properties.max_jerk;
    if (command_->linear_properties.tolerance == 0.0)
      command_->linear_properties.tolerance = default_command_->linear_properties.tolerance;
    if (command_->linear_properties.stopping_coeff == 0.0)
      command_->linear_properties.stopping_coeff = default_command_->linear_properties.stopping_coeff;
    if (command_->angular_properties.max_velocity == 0.0)
      command_->angular_properties.max_velocity = default_command_->angular_properties.max_velocity;
    if (command_->angular_properties.max_acceleration == 0.0)
      command_->angular_properties.max_acceleration = default_command_->angular_properties.max_acceleration;
    if (command_->angular_properties.max_jerk == 0.0)
      command_->angular_properties.max_jerk = default_command_->angular_properties.max_jerk;
    if (command_->angular_properties.tolerance == 0.0)
      command_->angular_properties.tolerance = default_command_->angular_properties.tolerance;
    if (command_->angular_properties.stopping_coeff == 0.0)
      command_->angular_properties.stopping_coeff = default_command_->angular_properties.stopping_coeff;
    
    end_time_ = command_->timeout + now();
    //is_sensor_detected_ = false;      // TODO ovde bi uzimali qos
    finished_ = false;
    result_->error = MoveDebug::ERROR_NONE;

    if(!update_odom_target_tf())
    {
      return false;
    }

    // Kickoff FSM
    lock_tf_odom_base_ = false;
    reversing_ = false;
    //reversing_coeff_ = 1.0;
    if (command_->mode & MoveCommand::Goal::REVERSE_MODE)
    {
      reversing_ = true;
      //reversing_coeff_ = -1.0;

    }
    if (command_->mode & MoveCommand::Goal::MODE_MOVE_TO_GOAL)
    {
      state_ = STATE_ROTATING_TOWARDS_GOAL;
      return true;
    }
    if (command_->mode & MoveCommand::Goal::MODE_ROTATE_AT_GOAL)
    {
      state_ = STATE_ROTATING_AT_GOAL;
      return true;
    }
    result_->error = MoveDebug::ERROR_MODE;
    return false;
  }

  double MoveActionServer::get_remaining_final_orientation(const tf2::Transform &tf_base_target)
  {
    return tf2::getYaw(tf_base_target.getRotation());
  }

  double MoveActionServer::get_remaining_heading(const tf2::Transform &tf_base_target)
  {
    if (reversing_)
      return atan2(-tf_base_target.getOrigin().y(), -tf_base_target.getOrigin().x());
    return atan2(tf_base_target.getOrigin().y(), tf_base_target.getOrigin().x());
  }

  double MoveActionServer::get_distance(const tf2::Transform &tf_base_target)
  {
    return sqrt(tf_base_target.getOrigin().x() * tf_base_target.getOrigin().x() + tf_base_target.getOrigin().y() * tf_base_target.getOrigin().y());
  }

  void MoveActionServer::state_rotating_towards_goal(const tf2::Transform &tf_base_target, const tf2::Transform &tf_odom_base, geometry_msgs::msg::TwistStamped *cmd_vel)
  {
    const bool first_iteration = (state_ != previous_state_);
    const double remaining_yaw = get_remaining_heading(tf_base_target);

    if (first_iteration)
    {
      debouncing_reset();
      const double distance_to_goal = get_distance(tf_base_target);
      if (distance_to_goal < command_->linear_properties.tolerance)
      {
        if(command_->mode & MoveCommand::Goal::MODE_ROTATE_AT_GOAL)
          state_ = STATE_ROTATING_AT_GOAL;
        else
        {
          state_ = STATE_IDLE;
        }
        return;
      }
      else
      {
        if (distance_to_goal < lock_tf_odom_base_threshold_)
        {
          // When a robot is very close to the goal we cannot use atan2(remaining_y, remaining_x) as the goal shifts during the rotation.
          lock_tf_odom_base_ = true;
          locked_tf_odom_base_ = tf_odom_base;
        }
        init_rotation(remaining_yaw);
      }
    }

    regulate_rotation(cmd_vel, remaining_yaw);
    if (std::fabs(remaining_yaw) < command_->angular_properties.tolerance)
    {
      if (now() >= debouncing_end_)
      {
        instant_robot_stop();
        lock_tf_odom_base_ = false;
        state_ = STATE_TRANSLATING;
      }
      return;
    }
    debouncing_reset();
  }

  void MoveActionServer::state_translating(const tf2::Transform &tf_base_target, geometry_msgs::msg::TwistStamped *cmd_vel)
  {
    const bool first_iteration = (state_ != previous_state_);
    if (first_iteration)
    {
      debouncing_reset();
      init_translation(tf_base_target.getOrigin().x());
    }

    regulate_translation(cmd_vel, tf_base_target.getOrigin().x(), tf_base_target.getOrigin().y());
    if (std::fabs(tf_base_target.getOrigin().x()) < command_->linear_properties.tolerance)
    {
      if (now() >= debouncing_end_)
      {
        instant_robot_stop();
        if (command_->mode & MoveCommand::Goal::MODE_ROTATE_AT_GOAL)
        {
          state_ = STATE_ROTATING_AT_GOAL;
        }
        else
        {
          state_ = STATE_IDLE;
        }
      }
      return;
    }
    debouncing_reset();
  }

  void MoveActionServer::state_rotating_at_goal(const tf2::Transform &tf_base_target, geometry_msgs::msg::TwistStamped *cmd_vel)
  {
    const bool first_iteration = (state_ != previous_state_);
    const double final_yaw = get_remaining_final_orientation(tf_base_target);

    if (first_iteration)
    {
      debouncing_reset();
      init_rotation(final_yaw);
    }

    regulate_rotation(cmd_vel, final_yaw);
    if (std::fabs(final_yaw) < command_->angular_properties.tolerance)
    {
      if (now() >= debouncing_end_)
      {
        instant_robot_stop();
        state_ = STATE_IDLE;
      }
      return;
    }
    debouncing_reset();
  }

  void MoveActionServer::state_stopping(const tf2::Transform &tf_base_target, geometry_msgs::msg::TwistStamped *cmd_vel)
  {
    const bool should_init = (state_ != previous_state_);

    if (should_init)
    {
      debouncing_reset();
      if (std::fabs(last_cmd_vel_.twist.linear.x) < min_velocity_tolerance_ || previous_state_ != STATE_TRANSLATING)
      {
        instant_robot_stop();
        state_ = STATE_IDLE;
        return;
      }
      init_stopping();
    }
    
    regulate_translation(cmd_vel, tf_base_target.getOrigin().x(), tf_base_target.getOrigin().y());
    if (std::fabs(last_cmd_vel_.twist.linear.x) < min_velocity_tolerance_)
    {
      if (now() >= debouncing_end_)
      {
        instant_robot_stop();
        state_ = STATE_IDLE;
      }
      return;
    }
    debouncing_reset();
  }

  void MoveActionServer::instant_robot_stop()
  {
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel_pub_->publish(cmd_vel);
  }

  void MoveActionServer::debouncing_reset()
  {
    debouncing_end_ = now() + debouncing_duration_;
  }

  // TODO prepisivanje greski
  void MoveActionServer::update()
  {
    // Timeout
    rclcpp::Duration time_remaining = end_time_ - now();
    if (time_remaining.seconds() < 0.0)  
    {
      if(result_->error == MoveDebug::ERROR_NONE)
        result_->error = MoveDebug::ERROR_TIMEOUT;
      state_ = STATE_STOPPING;
    }

    // Target in the base frame
    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(current_pose, *tf_, odom_frame_, robot_frame_))
    {
      result_->error = MoveDebug::ERROR_MISSING_TRANSFORM;
      previous_state_ = state_ = STATE_IDLE;
      finished_ = true;
      instant_robot_stop();
      return;
    }
    tf2::Transform tf_odom_base;
    tf2::convert(current_pose.pose, tf_odom_base);
    if (lock_tf_odom_base_)
    {
      tf_odom_base.getOrigin().setX(locked_tf_odom_base_.getOrigin().x());
      tf_odom_base.getOrigin().setY(locked_tf_odom_base_.getOrigin().y());
    }
    tf2::Transform tf_base_target = tf_odom_base.inverse() * tf_odom_target_;

    // cuvamo stanje da bi ga psole postavili za predhodno
    MoveStates previous_state = state_;

    // FSM
    auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
    switch (state_)
    {
    case STATE_ROTATING_TOWARDS_GOAL:
      state_rotating_towards_goal(tf_base_target, tf_odom_base, cmd_vel.get());
      break;
    case STATE_TRANSLATING:
      state_translating(tf_base_target, cmd_vel.get());
      break;
    case STATE_ROTATING_AT_GOAL:
      state_rotating_at_goal(tf_base_target, cmd_vel.get());
      break;
    case STATE_STOPPING:
      state_stopping(tf_base_target, cmd_vel.get());
      break;
    default:
      state_ = STATE_IDLE;
      break;
    }

    if (!(command_->ignore_obstacle) && state_ == STATE_TRANSLATING)
    {
      /*geometry_msgs::msg::PoseStamped current_pose;
      if (!nav2_util::getCurrentPose(current_pose, *tf_, odom_frame_, robot_frame_))
      {
        result_->error = MoveDebug::ERROR_MISSING_TRANSFORM;
        state_ = STATE_STOPPING;
      }*/

      geometry_msgs::msg::Pose2D pose2d;
      pose2d.x = current_pose.pose.position.x;
      pose2d.y = current_pose.pose.position.y;
      pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

      // TODO puna formula
      const double stopping_distance = stopping_distance_ + ((cmd_vel->twist.linear.x * cmd_vel->twist.linear.x) / (2 * command_->linear_properties.max_acceleration))
        + (std::fabs(cmd_vel->twist.linear.x) / update_rate_) + ((std::fabs(cmd_vel->twist.linear.x) * command_->linear_properties.max_acceleration) / (2 * command_->linear_properties.max_jerk));
      const double sim_position_change = sign(cmd_vel->twist.linear.x) * stopping_distance;
      pose2d.x += sim_position_change * cos(pose2d.theta);
      pose2d.y += sim_position_change * sin(pose2d.theta);

      bool is_collision_ahead = false;
      try
      {
        const double score = collision_checker_->scorePose(pose2d);
        if (score >= 254)
          is_collision_ahead = true;
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR_ONCE(get_logger(), "Collision checker failed: %s", e.what());
        if(result_->error == MoveDebug::ERROR_NONE)
          result_->error = MoveDebug::ERROR_OBSTACLE;
        state_ = STATE_STOPPING;
      }

      if (is_collision_ahead)
      {
        // TODO update_rate problem
        if(result_->error == MoveDebug::ERROR_NONE)
          result_->error = MoveDebug::ERROR_OBSTACLE;
        state_ = STATE_STOPPING;

        //state_msg_.error = mep3_msgs::msg::MoveState::ERROR_OBSTACLE;
        //state_ = mep3_msgs::msg::MoveState::STATE_IDLE;
        // update_debug_msg(tf_base_target);
        // state_pub_->publish(state_msg_);
        // return;
      }
    }
    
    if (state_ == STATE_IDLE)
    {
      finished_ = true;
      previous_state_ = state_;
      return;
    }

    previous_state_ = previous_state;
    
    last_cmd_vel_.twist = cmd_vel->twist;
    last_cmd_vel_.header = cmd_vel->header;
    cmd_vel_pub_->publish(std::move(cmd_vel));

    //update_debug_msg(tf_base_target);
    //state_pub_->publish(state_msg_);
  }

  void MoveActionServer::update_debug_msg(tf2::Transform &tf_base_target)
  {
    (void)tf_base_target;
    //state_msg_.state = state_;
    //state_msg_.distance_xy = get_distance(tf_base_target);
    //state_msg_.distance_x = tf_base_target.getOrigin().x();
    //state_msg_.distance_yaw = get_remaining_final_orientation(tf_base_target);
  }

  void MoveActionServer::init_rotation(double remaining_yaw)
  {
    if (rotation_ruckig_ != nullptr)
      delete rotation_ruckig_;

    rotation_ruckig_ = new ruckig::Ruckig<1>{1.0 / update_rate_};
    rotation_ruckig_input_.max_velocity = {command_->angular_properties.max_velocity};
    rotation_ruckig_input_.max_acceleration = {command_->angular_properties.max_acceleration};
    rotation_ruckig_input_.max_jerk = {command_->angular_properties.max_jerk};
    rotation_ruckig_input_.target_position = {0};
    rotation_ruckig_input_.current_velocity = {0};
    rotation_ruckig_input_.current_acceleration = {0};
    rotation_ruckig_input_.current_position = {remaining_yaw};
    rotation_ruckig_input_.control_interface = ruckig::ControlInterface::Position;
    rotation_ruckig_->update(rotation_ruckig_input_, rotation_ruckig_output_);

    next_step_yaw_ = 0;
  }

  void MoveActionServer::regulate_rotation(geometry_msgs::msg::TwistStamped *cmd_vel, double remaining_yaw)
  {
    const bool is_trajectory_finished = (rotation_ruckig_->update(rotation_ruckig_input_, rotation_ruckig_output_) == ruckig::Finished);
    next_step_yaw_ = remaining_yaw - rotation_ruckig_output_.new_position[0];
    cmd_vel->twist.angular.z = next_step_yaw_ * update_rate_;                    // dx/dt (dt[s] = 1 / update rate[Hz])

    cmd_vel->header.stamp = now();
    cmd_vel->header.frame_id = robot_frame_;

    if (!is_trajectory_finished)
      rotation_ruckig_output_.pass_to_input(rotation_ruckig_input_);
  }

  void MoveActionServer::init_translation(double remaining_x)
  {
    if (translation_ruckig_ != nullptr)
      delete translation_ruckig_;

    translation_ruckig_ = new ruckig::Ruckig<1>{1.0 / update_rate_};
    translation_ruckig_input_.max_velocity = {command_->linear_properties.max_velocity};
    translation_ruckig_input_.max_acceleration = {command_->linear_properties.max_acceleration};
    translation_ruckig_input_.max_jerk = {command_->linear_properties.max_jerk};
    translation_ruckig_input_.target_position = {0};
    translation_ruckig_input_.current_velocity = {0};
    translation_ruckig_input_.current_acceleration = {0};
    translation_ruckig_input_.current_position = {remaining_x};
    translation_ruckig_input_.control_interface = ruckig::ControlInterface::Position;
    translation_ruckig_->update(translation_ruckig_input_, translation_ruckig_output_);

    next_step_x_ = 0;
    next_step_y_ = 0;
  }

  void MoveActionServer::init_stopping()
  {
    if (translation_ruckig_ != nullptr)
      delete translation_ruckig_;

    translation_ruckig_ = new ruckig::Ruckig<1>{1.0 / update_rate_};

    translation_ruckig_input_.max_velocity = {command_->linear_properties.max_velocity * command_->linear_properties.stopping_coeff};
    translation_ruckig_input_.max_acceleration = {command_->linear_properties.max_acceleration * command_->linear_properties.stopping_coeff};
    translation_ruckig_input_.max_jerk = {command_->linear_properties.max_jerk * command_->linear_properties.stopping_coeff};
    translation_ruckig_input_.target_velocity = {0};
    translation_ruckig_input_.target_acceleration = {0};
    translation_ruckig_input_.control_interface = ruckig::ControlInterface::Velocity;
    translation_ruckig_->update(translation_ruckig_input_, translation_ruckig_output_);

    next_step_x_ = 0;
    next_step_y_ = 0;
  }

  void MoveActionServer::regulate_translation(geometry_msgs::msg::TwistStamped *cmd_vel, double remaining_x, double remaining_y)
  {
    const bool is_trajectory_finished = (translation_ruckig_->update(translation_ruckig_input_, translation_ruckig_output_) == ruckig::Finished);
    next_step_x_ = remaining_x - translation_ruckig_output_.new_position[0];
    cmd_vel->twist.linear.x = next_step_x_ * update_rate_;

    // TODO: Parameterize this + add kd
    cmd_vel->twist.angular.z = remaining_y * cmd_vel->twist.linear.x;

    cmd_vel->header.stamp = now();
    cmd_vel->header.frame_id = robot_frame_;

    if (!is_trajectory_finished)
      translation_ruckig_output_.pass_to_input(translation_ruckig_input_);
  }

  void MoveActionServer::ir_sensor_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (msg->data == 0)
    {
      is_sensor_detected_ = false;
    }
    else if (msg->data == 1)
    {
      RCLCPP_INFO(this->get_logger(), "Senzos is detected");
      is_sensor_detected_ = true;
    }
  }

  void MoveActionServer::nav2_init()
  {
    costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
        shared_from_this(), "local_costmap/costmap_raw");
    footprint_sub_ = std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
        shared_from_this(), "local_costmap/published_footprint", *tf_, robot_frame_);
    collision_checker_ =
        std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
            *costmap_sub_, *footprint_sub_, get_name());
  }

  MoveActionServer::MoveActionServer(const rclcpp::NodeOptions &options) : Node("move", options)
  {
    action_server_ = rclcpp_action::create_server<MoveCommand>(this, "move_action", 
      std::bind(&MoveActionServer::handle_goal, this,  std::placeholders::_1, std::placeholders::_2), 
      std::bind(&MoveActionServer::handle_cancel, this, std::placeholders::_1), 
      std::bind(&MoveActionServer::handle_accepted, this, std::placeholders::_1));
      
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 1);

    tf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    ir_sensor_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        "/ir_sensor_state", 10, std::bind(&MoveActionServer::ir_sensor_callback, this, std::placeholders::_1));

    // Read parameters
    declare_parameter("update_rate", rclcpp::ParameterValue(50));
    get_parameter("update_rate", update_rate_);

    double command_timeout;
    declare_parameter("command_timeout", rclcpp::ParameterValue(5.0));
    get_parameter("command_timeout", command_timeout);
    command_timeout_ = rclcpp::Duration::from_seconds(command_timeout);

    double debouncing_duration;
    declare_parameter("debouncing_duration", rclcpp::ParameterValue(0.1));
    get_parameter("debouncing_duration", debouncing_duration);
    debouncing_duration_ = rclcpp::Duration::from_seconds(debouncing_duration);

    declare_parameter("stopping_distance", rclcpp::ParameterValue(0.2));
    get_parameter("stopping_distance", stopping_distance_);

    declare_parameter("robot_frame", rclcpp::ParameterValue(std::string("base_link")));
    get_parameter("robot_frame", robot_frame_);

    declare_parameter("odom_frame", rclcpp::ParameterValue(std::string("odom")));
    get_parameter("odom_frame", odom_frame_);

    declare_parameter("debug_active", rclcpp::ParameterValue(false));
    get_parameter("debug_active", debug_node_active_);

    // Linear
    declare_parameter("linear.max_velocity", rclcpp::ParameterValue(0.1));
    get_parameter("linear.max_velocity", default_command_->linear_properties.max_velocity);

    declare_parameter("linear.max_acceleration", rclcpp::ParameterValue(1.5));
    get_parameter("linear.max_acceleration", default_command_->linear_properties.max_acceleration);

    declare_parameter("linear.max_jerk", rclcpp::ParameterValue(10.0));
    get_parameter("linear.max_jerk", default_command_->linear_properties.max_jerk);

    declare_parameter("linear.tolerance", rclcpp::ParameterValue(0.01));
    get_parameter("linear.tolerance", default_command_->linear_properties.tolerance);

    declare_parameter("linear.stopping_coeff", rclcpp::ParameterValue(4.0));
    get_parameter("linear.stopping_coeff", default_command_->linear_properties.stopping_coeff);

    declare_parameter("min_velocity_tolerance", rclcpp::ParameterValue(0.01));
    get_parameter("min_velocity_tolerance", min_velocity_tolerance_);

    //declare_parameter("linear.stuck_coeff", rclcpp::ParameterValue(0.1));
    //get_parameter("linear.stuck_coeff", linear_stuck_coeff_);

    // Angular
    declare_parameter("angular.max_velocity", rclcpp::ParameterValue(0.5));
    get_parameter("angular.max_velocity", default_command_->angular_properties.max_velocity);

    declare_parameter("angular.max_acceleration", rclcpp::ParameterValue(2.5));
    get_parameter("angular.max_acceleration", default_command_->angular_properties.max_acceleration);

    declare_parameter("angular.max_jerk", rclcpp::ParameterValue(10.0));
    get_parameter("angular.max_jerk", default_command_->angular_properties.max_jerk);

    declare_parameter("angular.tolerance", rclcpp::ParameterValue(0.03));
    get_parameter("angular.tolerance", default_command_->angular_properties.tolerance);

    declare_parameter("angular.stopping_coeff", rclcpp::ParameterValue(4.0));
    get_parameter("angular.stopping_coeff", default_command_->angular_properties.stopping_coeff);

    declare_parameter("lock_tf_odom_base_threshold", rclcpp::ParameterValue(0.15));
    get_parameter("lock_tf_odom_base_threshold", lock_tf_odom_base_threshold_);

    //declare_parameter("angular.stuck_coeff", rclcpp::ParameterValue(0.1));
    //get_parameter("angular.stuck_coeff", angular_stuck_coeff_);
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(zemo_navigation::MoveActionServer)