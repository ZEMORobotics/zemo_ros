// Minimal StraightLineMove implementation for testing plugin loading.
// This version intentionally does not implement navigation control.
// It returns zero velocity and avoids Regulated Pure Pursuit dependencies.

#include "zemo_navigation/controller_plugins/straight_line_move.hpp"
#include "nav2_core/controller_exceptions.hpp"

#define sign(x) (((x) > 0) - ((x) < 0))

static constexpr double PI_2 = M_PI * 2.0;

namespace zemo_navigation
{
/**
 * @brief Reverses the direction of the given angle by 180 degrees (PI radians).
 * @param angle Pointer to the angle in radians.
 */
inline static void angle_reverse(double* angle)
{    
  *angle >= 0 ? *angle-= M_PI : *angle+= M_PI;
}

/**
 * @brief Determine whether the robot should drive backwards to reach the target,
 * sets the reversing flag if the target is behind the robot
 * @param alpha Angle of the vector from the robot to the target in the global frame
 * @param robot_yaw Current robot yaw in the global frame
 * @return Boolean thaht defines if robot should go in reverse
 */
static inline bool shortest_path_direction(const double alpha, double robot_yaw)
{
  double angle = alpha - robot_yaw;

  if (angle > M_PI)
  {
    angle -= PI_2;
  }
  else if (angle < -M_PI)
  {
    angle += PI_2;
  }
  
  return (angle > M_PI_2 || angle < -M_PI_2);
}

geometry_msgs::msg::TwistStamped StraightLineMove::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*speed*/, nav2_core::GoalChecker * /*goal_checker*/)
{
  // resiti
  if (need_init_)
  {
    switch (state_)
    {
    case IDLE:
      init_rotation(pose);
      break;
    case ROTATING_TOWARDS_GOAL:
      init_translation(pose);
      break;
    default:
      RCLCPP_WARN(logger_, "Neocikavano stanje. Init FSM");
      break;
    }
  }

  if (state_ == TRANSLATING)  
  {
    // TODO puna formula za stopping distance
    //const double stopping_distance = (stopping_distance_ + ((speed.linear.x * speed.linear.x) / (2 * max_lin_acc_))
      //+ (std::fabs(speed.linear.x) / update_rate_) + ((std::fabs(speed.linear.x) * max_lin_acc_) / (2 * max_lin_jerk_)) * linear_vel_sign_);
    double stopping_distance = stopping_distance_ + ((ruckig_output_.new_velocity[0] * ruckig_output_.new_velocity[0]) / (2 * max_lin_acc_))
      + (std::fabs(ruckig_output_.new_velocity[0]) / update_rate_) + ((std::fabs(ruckig_output_.new_velocity[0]) * max_lin_acc_) / (2 * max_lin_jerk_));
    // for dynamic polygon implementation negative values a for backwords movement
    //RCLCPP_INFO(logger_,"STOPPING DISTANCE:%lf, IZRACUNAT STOPPING:%lf", stopping_distance_, stopping_distance);
    stopping_distance*=linear_vel_sign_;
    stopping_distance_publish(stopping_distance);
  }

  if (state_ == TRANSLATING && (collison_detetected_ || ir_sensor_detected_ || cancelling_))
  {
    //stopping function promeniti za realan rad sa robotom todo zemo
    init_stopping(ruckig_output_.new_velocity[0]);
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  regulate_velocity(pose, cmd_vel.get());
  
  geometry_msgs::msg::Twist msg;
  msg = cmd_vel->twist;
  turtle_pub_->publish(msg);

  return *cmd_vel;
}

void StraightLineMove::setPlan(const nav_msgs::msg::Path & path)
{
  // Intentionally implemented only for one pose goal. todo zemo
  target_pose_ = path.poses[0];
  need_init_ = true;
}


void StraightLineMove::init_stopping(const double & speed)
{
  if (ruckig_ != nullptr)
  {
    delete ruckig_;
  }

  ruckig_ = new ruckig::Ruckig<1>{1.0 / update_rate_};
  ruckig_input_.max_velocity = {max_lin_vel_ * stopping_coeff_};
  ruckig_input_.max_acceleration = {max_lin_acc_ * stopping_coeff_};
  ruckig_input_.max_jerk = {max_lin_jerk_ * stopping_coeff_};
  ruckig_input_.target_velocity = {0.0};
  ruckig_input_.target_acceleration = {0.0};
  ruckig_input_.current_velocity = {speed};
  //ruckig_input_.current_acceleration = {0};
  ruckig_input_.control_interface = ruckig::ControlInterface::Velocity;

  state_ = STOPPING;
}

void StraightLineMove::init_rotation(const geometry_msgs::msg::PoseStamped current_pose)
{
  double dx = target_pose_.pose.position.x - current_pose.pose.position.x;
  double dy = target_pose_.pose.position.y - current_pose.pose.position.y;

  const double alpha = std::atan2(dy, dx);
  double robot_yaw = tf2::getYaw(current_pose.pose.orientation);

  reversing_ = shortest_path_direction(alpha, robot_yaw);
  if (reversing_)
  {
    angle_reverse(&robot_yaw);
    linear_vel_sign_ = -1.0;
  }
  else
  {
    linear_vel_sign_ = 1.0;
  }

  double rotation_angle1 = (alpha - robot_yaw);
  // if (shortest_path_direction(alpha, robot_yaw))
  // {
  //   angle_reverse(&rotation_angle1);
  // }

  // base link transform
  const double gx = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  const double gy = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

  double rotation_angle = std::atan2(gy, gx);
  angular_vel_sign_ = sign(rotation_angle);
  rotation_angle = std::fabs(rotation_angle);

  // RCLCPP_INFO(logger_, "Oduzimanje: %lf, base_link: %lf je XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", rotation_angle1, rotation_angle);

  if (ir_sensor_detected_ || collison_detetected_)
  {
    // vec na destinaciji ili uopste ne krecemo todo zemo
    // promeniti impelmentaciju controller servera kako bi ovo postigao
    throw nav2_core::ControllerException("Unable to init movement rotation!");
  }
  else if (rotation_angle < yaw_tolerance_)
  {
    /* code */
    RCLCPP_INFO(logger_, "No need for rotation!");
    init_translation(current_pose);
    return;
  }

  RCLCPP_INFO(logger_, "UGAO ROBOTA JE: %lf, ROTTAION ANGLE je = %lf, linear_vel_sign = %lf, angular_vel_sign = %lf", robot_yaw, rotation_angle, linear_vel_sign_, angular_vel_sign_);

  if (ruckig_ != nullptr)
  {
    delete ruckig_;
  }
  
  ruckig_ = new ruckig::Ruckig<1>{1.0 / update_rate_};
  ruckig_input_.max_velocity = {max_lin_vel_};
  ruckig_input_.max_acceleration = {max_lin_acc_};
  ruckig_input_.max_jerk = {max_lin_jerk_};
  ruckig_input_.target_position = {rotation_angle};
  ruckig_input_.current_velocity = {0};
  ruckig_input_.current_acceleration = {0};
  ruckig_input_.current_position = {0};
  ruckig_input_.control_interface = ruckig::ControlInterface::Position;
  //ruckig_->update(ruckig_input_, ruckig_output_);

  state_ = ROTATING_TOWARDS_GOAL;
  need_init_ = false;
}


void StraightLineMove::init_translation(const geometry_msgs::msg::PoseStamped current_pose)
{
  double dx = target_pose_.pose.position.x - current_pose.pose.position.x;
  double dy = target_pose_.pose.position.y - current_pose.pose.position.y;

  // double robot_yaw = tf2::getYaw(current_pose.pose.orientation);

  // base link transform
  // const double gx = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  // const double gy = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

  if (ir_sensor_detected_ || collison_detetected_)
  {
    // vec na destinaciji ili uopste ne krecemo todo zemo
    // promeniti impelmentaciju controller servera kako bi ovo postigao
    throw nav2_core::ControllerException("Unable to init movement translation!");
  }
  // else if (dy > 0.1)    // todo zemo
  // {
  //   throw nav2_core::ControllerException("Not rotated even if should!");
  // }

   RCLCPP_INFO(logger_, "DX: %lf, DY: %lf je XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", dx, dy);
  // RCLCPP_INFO(logger_, "UGAO ROBOTA JE: %lf, ROTTAION ANGLE je = %lf, linear_vel_sign = %lf", robot_yaw, rotation_angle, linear_vel_sign_);

  if (ruckig_ != nullptr)
  {
    delete ruckig_;
  }
  
  ruckig_ = new ruckig::Ruckig<1>{1.0 / update_rate_};
  ruckig_input_.max_velocity = {max_lin_vel_};
  ruckig_input_.max_acceleration = {max_lin_acc_};
  ruckig_input_.max_jerk = {max_lin_jerk_};
  ruckig_input_.target_position = {std::fabs(dx)};
  ruckig_input_.current_velocity = {0};
  ruckig_input_.current_acceleration = {0};
  ruckig_input_.current_position = {0};
  ruckig_input_.control_interface = ruckig::ControlInterface::Position;
  //ruckig_->update(ruckig_input_, ruckig_output_);

  state_ = TRANSLATING;
  need_init_ = false;
}

void StraightLineMove::regulate_velocity(const geometry_msgs::msg::PoseStamped /*current_pose*/, geometry_msgs::msg::TwistStamped *cmd_vel)
{
  const bool is_trajectory_finished = (ruckig_->update(ruckig_input_, ruckig_output_) == ruckig::Finished);
  switch (state_)
  {
  case ROTATING_TOWARDS_GOAL:
    {
    // double remaining = remaining_angle(current_pose);
    double next_step = ruckig_output_.new_position[0] - ruckig_input_.current_position[0];
    //double next_step = remaining - ruckig_output_.new_position[0];

    //RCLCPP_INFO(logger_, "new: %lf, cur: %lf je ", ruckig_output_.new_position[0], ruckig_input_.current_position[0]);
    //RCLCPP_INFO(logger_, "next step: %lf, update_rate: %lf je XXX", next_step, update_rate_);
    
    // dx/dt (dt[s] = 1 / update rate[Hz])
    cmd_vel->twist.angular.z = next_step * update_rate_ * angular_vel_sign_;
    // RCLCPP_INFO(logger_, "Nalazimo se u rotation i state je znak: %lf, a cmd_vel %lf, nest_step je %lf", angular_vel_sign_, cmd_vel->twist.angular.z, next_step);
    }
    break;
  case TRANSLATING:
    {
    //double remaining = remaining_distance(current_pose);
    double next_step = ruckig_output_.new_position[0] - ruckig_input_.current_position[0];
    //double next_step = remaining - ruckig_output_.new_position[0];

    //RCLCPP_INFO(logger_, "new: %lf, cur: %lf je ", ruckig_output_.new_position[0], ruckig_input_.current_position[0]);
    //RCLCPP_INFO(logger_, "next step: %lf, update_rate: %lf je XXX", next_step, update_rate_);
    
    // dx/dt (dt[s] = 1 / update rate[Hz])
    cmd_vel->twist.linear.x = next_step * update_rate_ * linear_vel_sign_;
    // RCLCPP_INFO(logger_, "Nalazimo se u translaciji i state je %d, a cmd_vel %lf", state_, cmd_vel->twist.linear.x);
    }
    break;
  case STOPPING:
    {
    cmd_vel->twist.linear.x = ruckig_output_.new_velocity[0] * linear_vel_sign_;
    RCLCPP_INFO(logger_, "Nalazimo se u stopping i state je %d, a cmd_vel %lf", state_, cmd_vel->twist.linear.x);
    }
    break;
  default:
    RCLCPP_WARN(logger_, "Greska u regulate velocity!");
    break;
  }

  cmd_vel->header.stamp = clock_->now();
  cmd_vel->header.frame_id = "base_link";

  if (!is_trajectory_finished)
  {
    ruckig_output_.pass_to_input(ruckig_input_);
  }
  // u slucaju da smo u stopping stanju ovako proveravmo kraj 
  else if (state_ == ROTATING_TOWARDS_GOAL)
  {
    need_init_ = true;
  }
  else if (cancelling_)
  {
    finished_cancelling_ = true;
  }
  else
  {
    throw nav2_core::ControllerException("Stopped becouse of detetction");
  }
}

double StraightLineMove::remaining_angle(const geometry_msgs::msg::PoseStamped current_pose)
{
  double dx = target_pose_.pose.position.x - current_pose.pose.position.x;
  double dy = target_pose_.pose.position.y - current_pose.pose.position.y;
  const double alpha = std::atan2(dy, dx);
  double robot_yaw = tf2::getYaw(current_pose.pose.orientation);

  if (reversing_)
  {
    angle_reverse(&robot_yaw);
  }

  return std::fabs(alpha - robot_yaw);
}

double StraightLineMove::remaining_distance(const geometry_msgs::msg::PoseStamped current_pose)
{
  const double dx = target_pose_.pose.position.x - current_pose.pose.position.x;
  // const double dy = target_pose_.pose.position.y - current_pose.pose.position.y;
  // double robot_yaw = tf2::getYaw(current_pose.pose.orientation);

  // if (reversing_)
  // {
  //   angle_reverse(&robot_yaw);
  // }

  // racunamo kao da smo u pravi da se y ne menja i da je ne bitan sto znaci da nam ni dy ne treba
  //const double gx = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  // const double gy = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

  return std::fabs(dx);
}

void StraightLineMove::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock node!");
  }

  node_ = parent;
  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  collison_detetected_ = false;
  ir_sensor_detected_ = false;
  need_init_ = true;
  reversing_ = false;
  state_ = IDLE;

  //-------------------------------------------------
  max_lin_vel_ = 5.0;
  max_lin_acc_ = 10.0;
  max_lin_jerk_ = 100.0;
  max_ang_vel_ = 5.0;
  max_ang_acc_ = 10.0;
  max_ang_jerk_ = 100.0;
  //-------------------------------------------------

  // params
  node->get_parameter("min_x_velocity_threshold", min_lin_vel_tolerance_);
  node->get_parameter("min_theta_velocity_threshold", min_ang_vel_tolerance_);

  node->get_parameter("controller_frequency", update_rate_);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".collison_disatnce_topic", rclcpp::PARAMETER_STRING);
  collison_disatnce_topic_ = node->get_parameter(plugin_name_ + ".collison_disatnce_topic").as_string();

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".stopping_distance", rclcpp::ParameterValue(0.2));
  stopping_distance_ = node->get_parameter(plugin_name_ + ".stopping_distance").as_double();

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".stopping_coeff", rclcpp::ParameterValue(4.0));
  stopping_coeff_ = node->get_parameter(plugin_name_ + ".stopping_coeff").as_double();

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".yaw_tolerance", rclcpp::ParameterValue(0.01));
  yaw_tolerance_ = node->get_parameter(plugin_name_ + ".yaw_tolerance").as_double();
  // ---------------------------------

  turtle_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 1);
  stop_dist_pub_ = node->create_publisher<std_msgs::msg::Float32>(collison_disatnce_topic_, 1);    // zemo todo zemo qs

  collison_detector_subscription_ = node->create_subscription<nav2_msgs::msg::CollisionDetectorState>(
    "/collision_detector_state", 10, std::bind(&StraightLineMove::collision_detector_callback, this, std::placeholders::_1));
  
  ir_sensor_subscription_ = node->create_subscription<std_msgs::msg::Int32>(
    "/ir_sensor_state", 10, std::bind(&StraightLineMove::ir_sensor_callback, this, std::placeholders::_1));
}

void StraightLineMove::cleanup()
{
  stop_dist_pub_.reset();
  turtle_pub_.reset();

  state_ = IDLE;

  // ----------------------
  global_path_pub_.reset();
  carrot_pub_.reset();
  curvature_carrot_pub_.reset();
  is_rotating_to_heading_pub_.reset();
  // --------------------
}

bool StraightLineMove::cancel()
{
  // zalika od reset todo zemo
  if(state_ == TRANSLATING) 
  { 
    cancelling_ = true;
  }

  return finished_cancelling_;
}

void StraightLineMove::setSpeedLimit(
  const double & /*speed_limit*/,
  const bool & /*percentage*/)
{
  // Intentionally unused in the minimal test implementation.
}

void StraightLineMove::reset()
{
  //-----------------------------
  cancelling_ = false;
  finished_cancelling_ = false;
  // has_reached_xy_tolerance_ = false;
  //------------------------------
  collison_detetected_ = false;
  ir_sensor_detected_ = false;
  reversing_ = false;
  need_init_ = true;
  state_ = IDLE;
}

void StraightLineMove::activate()
{
  // No publishers are created in this minimal test implementation.
  stop_dist_pub_->on_activate();
  turtle_pub_->on_activate();
}

void StraightLineMove::deactivate()
{
  // No publishers are created in this minimal test implementation.
  stop_dist_pub_->on_deactivate();
  turtle_pub_->on_deactivate();
}

void StraightLineMove::stopping_distance_publish(const double stopping_distance)
{
  if (stop_dist_pub_->is_activated())
  {
    std_msgs::msg::Float32 msg;
    msg.data = stopping_distance;
    stop_dist_pub_->publish(msg);
  }
}

void StraightLineMove::ir_sensor_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  if (msg->data == 1)
  {
    ir_sensor_detected_ = true;
  }
  else if (msg->data == 0)
  {
    ir_sensor_detected_ = false;
  }
}

void StraightLineMove::collision_detector_callback(const nav2_msgs::msg::CollisionDetectorState::SharedPtr msg)
{
  if (!(msg->polygons.empty() || msg->detections.empty()))
  {
    // zemo todo implemented for only one polygon
    if (msg->detections.front())
    {
      collison_detetected_ = true;
    }
    else
    {
      collison_detetected_ = false;
    }
  }
  else
  {
    RCLCPP_WARN(logger_, "Prazan collison detector msg!");
  }
}

}  // namespace zemo_navigation

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(zemo_navigation::StraightLineMove, nav2_core::Controller)
