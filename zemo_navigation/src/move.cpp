#include "zemo_navigation/move.hpp"

#define sign(x) (((x) > 0) - ((x) < 0))

namespace zemo_navigation
{
  rclcpp_action::GoalResponse MoveActionServer::handle_goal(const rclcpp_action::GoalUUID &uuid , std::shared_ptr<const move_command::Goal> goal)
  {
    (void)uuid;
    if(!update_odom_target_tf())
    {
      return rclcpp_action::GoalResponse::REJECT;
    }

    // init command
    command_->header = goal->header;
    command_->odom_frame = goal->odom_frame;
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
    (void)goal_handle;
    auto execute_thread = [this, goal_handle](){ return this->execute(goal_handle);};
    std::thread{execute_thread}.detach();
  }

  void MoveActionServer::execute(const std::shared_ptr<GoalHandleMove> goal_handle)
  {

  }

  bool MoveActionServer::update_odom_target_tf()
  {
    tf2::Transform tf_global_target;
    tf_global_target.setOrigin(tf2::Vector3(command_->target.x, command_->target.y, 0.0));
    tf_global_target.setRotation(tf2::Quaternion(
        tf2::Vector3(0, 0, 1), command_->target.theta));

    geometry_msgs::msg::TransformStamped tf_global_odom_message;
    try
    {
        tf_global_odom_message = tf_->lookupTransform(command_->header.frame_id, command_->odom_frame, command_->header.stamp);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Initial global_frame -> command_->odom_frame is not available.");

        //state_msg_.error = mep3_msgs::msg::MoveState::ERROR_MISSING_TRANSFORM;
        state_ = move_state::STATE_IDLE;
        //state_pub_->publish(state_msg_);

        return false;
    }

    tf2::Transform tf_global_odom;
    tf2::convert(tf_global_odom_message.transform, tf_global_odom);
    tf_odom_target_ = tf_global_odom.inverse() * tf_global_target;
    target_updated_ = true;

    return true;
  }

  bool MoveActionServer::init_move()
  {
    end_time_ = command_->timeout + now();
    start_action_time_ = now();
    //is_sensor_detected_ = false;

    // Apply defaults
    if (command_->header.frame_id == "")
      command_->header.frame_id = "map";
    if (command_->odom_frame == "")
      command_->odom_frame = "odom";
    if (command_->linear_properties.max_velocity == 0.0)
      command_->linear_properties.max_velocity = default_command_->linear_properties.max_velocity;
    if (command_->linear_properties.max_acceleration == 0.0)
      command_->linear_properties.max_acceleration = default_command_->linear_properties.max_acceleration;
    if (command_->linear_properties.tolerance == 0.0)
      command_->linear_properties.tolerance = default_command_->linear_properties.tolerance;
    if (command_->angular_properties.max_velocity == 0.0)
      command_->angular_properties.max_velocity = default_command_->angular_properties.max_velocity;
    if (command_->angular_properties.max_acceleration == 0.0)
      command_->angular_properties.max_acceleration = default_command_->angular_properties.max_acceleration;
    if (command_->angular_properties.tolerance == 0.0)
      command_->angular_properties.tolerance = default_command_->angular_properties.tolerance;

    // Kickoff FSM
    lock_tf_odom_base_ = false;
    reversing_ = false;
    if (command_->mode & move_command::Goal::REVERSE_MODE)
    {
      /* treba dodati, okrenuti target, bez return, i mozda -1.0 ali to pitaj vladu */
      reversing_ = true;
    }
    if (command_->mode & move_command::Goal::MODE_MOVE_TO_GOAL)
    {
      state_ = move_state::STATE_ROTATING_TOWARDS_GOAL;
      return true;
    }
    if (command_->mode & move_command::Goal::MODE_ROTATE_AT_GOAL)
    {
      state_ = move_state::STATE_ROTATING_AT_GOAL;
      return true;
    }
    RCLCPP_ERROR(this->get_logger(), "Invalid MoveCommand, invalide mode parameter");
    return false;
  }

  double MoveActionServer::get_diff_final_orientation(const tf2::Transform &tf_base_target)
  {
    return tf2::getYaw(tf_base_target.getRotation());
  }

  double MoveActionServer::get_diff_heading(const tf2::Transform &tf_base_target)
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
    const double diff_yaw = get_diff_heading(tf_base_target);

    if (first_iteration)
    {
      debouncing_reset();
      const double distance_to_goal = get_distance(tf_base_target);
      if (distance_to_goal < command_->linear_properties.tolerance)
      {
        // In case we are already at the goal we skip rotation towards the goal and translation.
        state_ = move_state::STATE_ROTATING_AT_GOAL;
        return;
      }
      else
      {
        if (distance_to_goal < lock_tf_odom_base_threshold_)
        {
          // When a robot is very close to the goal we cannot use atan2(diff_y, diff_x) as the goal shifts during the rotation.
          lock_tf_odom_base_ = true;
          locked_tf_odom_base_ = tf_odom_base;
        }
        init_rotation(diff_yaw);
      }
    }

    regulate_rotation(cmd_vel, diff_yaw);
    if (abs(diff_yaw) < command_->angular_properties.tolerance)
    {
      if (now() >= debouncing_end_)
      {
        instant_robot_stop();
        lock_tf_odom_base_ = false;
        state_ = move_state::STATE_TRANSLATING;
        debouncing_reset();
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

  void MoveActionServer::state_translating(const tf2::Transform &tf_base_target, geometry_msgs::msg::TwistStamped *cmd_vel)
  {
    const bool first_iteration = (state_ != previous_state_);
    if (first_iteration)
    {
      debouncing_reset();
      init_translation(tf_base_target.getOrigin().x(), tf_base_target.getOrigin().y());
    }

    regulate_translation(cmd_vel, tf_base_target.getOrigin().x(), tf_base_target.getOrigin().y());
    if (abs(tf_base_target.getOrigin().x()) < command_->linear_properties.tolerance)
    {
      if (now() >= debouncing_end_)
      {
        instant_robot_stop();
        if (command_->mode & move_command::Goal::MODE_ROTATE_AT_GOAL)
        {
          state_ = move_state::STATE_ROTATING_AT_GOAL;
          debouncing_reset();
        }
        else
        {
          state_ = move_state::STATE_IDLE;
          return;
        }
      }
      return;
    }
    debouncing_reset();
  }

  void MoveActionServer::state_rotating_at_goal(const tf2::Transform &tf_base_target, geometry_msgs::msg::TwistStamped *cmd_vel)
  {
    const bool first_iteration = (state_ != previous_state_);
    const double final_yaw = get_diff_final_orientation(tf_base_target);

    if (first_iteration)
    {
      debouncing_reset();
      init_rotation(final_yaw);
    }

    regulate_rotation(cmd_vel, final_yaw);
    if (abs(final_yaw) < command_->angular_properties.tolerance)
    {
      if (now() >= debouncing_end_)
      {
        instant_robot_stop();
        debouncing_reset();
        state_ = move_state::STATE_IDLE;
      }
      return;
    }
    debouncing_reset();
  }

  void MoveActionServer::update()
  {
    previous_state_ = state_;

    if (state_ == move_state::STATE_IDLE)
    {
      return;
    }

    // Timeout
    rclcpp::Duration time_remaining = end_time_ - now();
    if (time_remaining.seconds() < 0.0 && rclcpp::Duration(command_->timeout).seconds() > 0.0)
    {
      instant_robot_stop();
      RCLCPP_WARN(this->get_logger(), "ERROR_TIMEOUT, allowed time exceeded for the Move action");
      state_ = move_state::STATE_IDLE;
      return;
    }

    // Target in the base frame
    geometry_msgs::msg::PoseStamped tf_odom_base_message;
    if (!nav2_util::getCurrentPose(tf_odom_base_message, *tf_, command_->odom_frame, robot_frame_))
    {
      RCLCPP_ERROR(this->get_logger(), "Initial odom_frame -> base frame transform is not available.");
      state_ = move_state::STATE_IDLE;
      return;
    }
    tf2::Transform tf_odom_base;
    tf2::convert(tf_odom_base_message.pose, tf_odom_base);
    if (lock_tf_odom_base_)
    {
      tf_odom_base.getOrigin().setX(locked_tf_odom_base_.getOrigin().x());
      tf_odom_base.getOrigin().setY(locked_tf_odom_base_.getOrigin().y());
    }
    tf2::Transform tf_base_target = tf_odom_base.inverse() * tf_odom_target_;

    // FSM
    //state_msg_.error = mep3_msgs::msg::MoveState::ERROR_NONE;
    auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
    switch (state_)
    {
    case move_state::STATE_ROTATING_TOWARDS_GOAL:
      state_rotating_towards_goal(tf_base_target, tf_odom_base, cmd_vel.get());
      break;
    case move_state::STATE_TRANSLATING:
      state_translating(tf_base_target, cmd_vel.get());
      break;
    case move_state::STATE_ROTATING_AT_GOAL:
      state_rotating_at_goal(tf_base_target, cmd_vel.get());
      break;
    case move_state::STATE_STOPPING:
      // TODO
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "FSM invalid state error!");

      break;
    }

    // Detect stuck
    /*const double planned_rotation_velocity = std::max(rotation_ruckig_output_.new_velocity[0], 0.01);
    const double planned_translation_velocity = std::max(translation_ruckig_output_.new_velocity[0], 0.01);
    // std::cout << "rotation: " << last_error_yaw_ << " translation: " << last_error_x_ << std::endl;
    int64_t elapsed_time_ms = (now() - start_action_time_).nanoseconds() / 1000000;
    if (elapsed_time_ms > 300 && state_ == move_state::STATE_TRANSLATING && abs(last_error_x_) > abs(planned_translation_velocity * linear_stuck_coeff_))
    {
      instant_robot_stop();
      RCLCPP_WARN(get_logger(), "Stuck detected, stopping translating...");

      state_msg_.error = mep3_msgs::msg::MoveState::ERROR_STUCK;

      state_ = mep3_msgs::msg::MoveState::STATE_IDLE;
      update_state_msg(tf_base_target);
      state_pub_->publish(state_msg_);
    }

    if (elapsed_time_ms > 300 && (state_ == mep3_msgs::msg::MoveState::STATE_ROTATING_TOWARDS_GOAL || state_ == mep3_msgs::msg::MoveState::STATE_ROTATING_AT_GOAL) && abs(last_error_yaw_) > abs(planned_rotation_velocity * angular_stuck_coeff_))
    {
      instant_robot_stop();
      RCLCPP_WARN(get_logger(), "Stuck detected, stopping rotation...");

      state_msg_.error = mep3_msgs::msg::MoveState::ERROR_STUCK;

      state_ = mep3_msgs::msg::MoveState::STATE_IDLE;
      update_state_msg(tf_base_target);
      state_pub_->publish(state_msg_);
    }

    if (command_->enable_sensors && is_sensor_detected_)
    {
      RCLCPP_WARN(get_logger(), "Sensor collision Ahead!");

      state_msg_.error = mep3_msgs::msg::MoveState::ERROR_OBSTACLE;

      state_ = mep3_msgs::msg::MoveState::STATE_IDLE;
    }
*/
    // Stop if there is a collision
    if (!command_->ignore_obstacle)
    {
      geometry_msgs::msg::PoseStamped current_pose;
      if (!nav2_util::getCurrentPose(current_pose, *tf_, odom_frame_, robot_frame_))
      {
        RCLCPP_ERROR(get_logger(), "Current robot pose is not available.");
        state_ = move_state::STATE_IDLE;
        return;
      }

      geometry_msgs::msg::Pose2D pose2d;
      pose2d.x = current_pose.pose.position.x;
      pose2d.y = current_pose.pose.position.y;
      pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

      // puna formula
      const double stopping_distance = stopping_distance_ + (cmd_vel->twist.linear.x * cmd_vel->twist.linear.x) / (2 * 2.5);        // formula
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
      }

      if (is_collision_ahead)
      {
        // instant_robot_stop();
        RCLCPP_WARN(get_logger(), "ERROR_OBSTACLE, collision detected!");

        //state_msg_.error = mep3_msgs::msg::MoveState::ERROR_OBSTACLE;

        //state_ = mep3_msgs::msg::MoveState::STATE_IDLE;
        // update_state_msg(tf_base_target);
        // state_pub_->publish(state_msg_);
        // return;
      }
    }
    
    cmd_vel_pub_->publish(std::move(cmd_vel));

    update_state_msg(tf_base_target);
    //state_pub_->publish(state_msg_);
  }

  void MoveActionServer::update_state_msg(tf2::Transform &tf_base_target)
  {
    //state_msg_.state = state_;
    //state_msg_.distance_xy = get_distance(tf_base_target);
    //state_msg_.distance_x = tf_base_target.getOrigin().x();
    //state_msg_.distance_yaw = get_diff_final_orientation(tf_base_target);
  }

  void MoveActionServer::init_rotation(double diff_yaw)
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
    rotation_ruckig_input_.current_position = {diff_yaw};
    rotation_ruckig_input_.control_interface = ruckig::ControlInterface::Position;
    rotation_ruckig_->update(rotation_ruckig_input_, rotation_ruckig_output_);

    last_error_yaw_ = 0;
  }

  void MoveActionServer::regulate_rotation(geometry_msgs::msg::TwistStamped *cmd_vel, double diff_yaw)
  {
    const bool is_trajectory_finished = (rotation_ruckig_->update(rotation_ruckig_input_, rotation_ruckig_output_) == ruckig::Finished);
    last_error_yaw_ = diff_yaw - rotation_ruckig_output_.new_position[0];
    cmd_vel->twist.angular.z = last_error_yaw_ * update_rate_;                    // dx/dt (dt[s] = 1 / update rate[Hz])

    if (!is_trajectory_finished)
      rotation_ruckig_output_.pass_to_input(rotation_ruckig_input_);
  }

  void MoveActionServer::init_translation(double diff_x, double diff_y)
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
    translation_ruckig_input_.current_position = {diff_x};
    translation_ruckig_input_.control_interface = ruckig::ControlInterface::Position;
    translation_ruckig_->update(translation_ruckig_input_, translation_ruckig_output_);

    last_error_x_ = 0;
    last_error_y_ = 0;
  }

  void MoveActionServer::regulate_translation(geometry_msgs::msg::TwistStamped *cmd_vel, double diff_x, double diff_y)
  {
    const bool is_trajectory_finished = (translation_ruckig_->update(translation_ruckig_input_, translation_ruckig_output_) == ruckig::Finished);
    last_error_x_ = diff_x - translation_ruckig_output_.new_position[0];
    cmd_vel->twist.linear.x = last_error_x_ * update_rate_;

    // TODO: Parameterize this + add kd
    cmd_vel->twist.angular.z = diff_y * cmd_vel->twist.linear.x * 1.0;

    if (!is_trajectory_finished)
      translation_ruckig_output_.pass_to_input(translation_ruckig_input_);
  }

  void MoveActionServer::param_decleration()
  {
    // Read parameters
    declare_parameter("update_rate", rclcpp::ParameterValue(50));
    get_parameter("update_rate", update_rate_);

    double command_timeout;
    declare_parameter("command_timeout", rclcpp::ParameterValue(0.5));
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

    // Linear
    declare_parameter("linear.max_velocity", rclcpp::ParameterValue(0.1));
    get_parameter("linear.max_velocity", default_command_->linear_properties.max_velocity);

    declare_parameter("linear.max_acceleration", rclcpp::ParameterValue(1.5));
    get_parameter("linear.max_acceleration", default_command_->linear_properties.max_acceleration);

    declare_parameter("angular.max_acceleration", rclcpp::ParameterValue(10));
    get_parameter("angular.max_acceleration", default_command_->linear_properties.max_jerk);

    declare_parameter("linear.tolerance", rclcpp::ParameterValue(0.01));
    get_parameter("linear.tolerance", default_command_->linear_properties.tolerance);

    //declare_parameter("linear.stuck_coeff", rclcpp::ParameterValue(0.1));
    //get_parameter("linear.stuck_coeff", linear_stuck_coeff_);

    // Angular
    declare_parameter("angular.max_velocity", rclcpp::ParameterValue(0.5));
    get_parameter("angular.max_velocity", default_command_->angular_properties.max_velocity);

    declare_parameter("angular.max_acceleration", rclcpp::ParameterValue(0.1));
    get_parameter("angular.max_acceleration", default_command_->angular_properties.max_acceleration);

    declare_parameter("angular.max_acceleration", rclcpp::ParameterValue(10));
    get_parameter("angular.max_acceleration", default_command_->angular_properties.max_jerk);

    declare_parameter("angular.tolerance", rclcpp::ParameterValue(0.03));
    get_parameter("angular.tolerance", default_command_->angular_properties.tolerance);

    declare_parameter("lock_tf_odom_base_threshold", rclcpp::ParameterValue(0.15));
    get_parameter("odom_frame", lock_tf_odom_base_threshold_);

    //declare_parameter("angular.stuck_coeff", rclcpp::ParameterValue(0.1));
    //get_parameter("angular.stuck_coeff", angular_stuck_coeff_);
  }

  void MoveActionServer::sensor_callback(const std_msgs::msg::Int32::SharedPtr msg)
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

  MoveActionServer::MoveActionServer(const rclcpp::NodeOptions &options) : Node("move", options)
  {
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 1);
    state_pub_ = create_publisher<move_state>("~/state", 1);
    action_server_ = rclcpp_action::create_server<move_command>(this, "move_action", 
      std::bind(&MoveActionServer::handle_goal, this,  std::placeholders::_1, std::placeholders::_2), 
      std::bind(&MoveActionServer::handle_cancel, this, std::placeholders::_1), 
      std::bind(&MoveActionServer::handle_accepted, this, std::placeholders::_1));

    tf_ =
        std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_);

    costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
        shared_from_this(), "local_costmap/costmap_raw");
    footprint_sub_ = std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
        shared_from_this(), "local_costmap/published_footprint", *tf_, robot_frame_);
    collision_checker_ =
        std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
            *costmap_sub_, *footprint_sub_, get_name());

    ir_sensor_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        "/ir_sensor_state", 10, std::bind(&MoveActionServer::sensor_callback, this, std::placeholders::_1));

  }

  void MoveActionServer::debouncing_reset()
  {
    debouncing_end_ = now() + debouncing_duration_;
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(zemo_navigation::MoveActionServer)