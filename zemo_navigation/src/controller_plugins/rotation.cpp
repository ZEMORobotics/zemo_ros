// Minimal Rotation implementation for testing plugin loading.
// This version intentionally does not implement navigation control.
// It returns zero velocity and avoids Regulated Pure Pursuit dependencies.

#include "zemo_navigation/controller_plugins/rotation.hpp"
#include "nav2_core/controller_exceptions.hpp"

#define sign(x) (((x) > 0) - ((x) < 0))

static constexpr double PI_2 = M_PI * 2.0;

namespace zemo_navigation
{
/**
 * @brief Reverses the direction of the given angle by 180 degrees (PI radians).
 * @param angle Pointer to the angle in radians.
 */
inline static void reverse_M_PI(double* angle)
{    
  *angle >= 0 ? *angle-= M_PI : *angle+= M_PI;
}

/**
 * @brief Reverses the direction of the given angle by 360 degrees (2 * PI radians). If we want a longer route.
 * @param angle Pointer to the angle in radians.
 */
inline static void reverse_2_M_PI(double* angle)
{    
  *angle >= 0 ? *angle-= PI_2 : *angle+= PI_2;
}

geometry_msgs::msg::TwistStamped Rotation::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*speed*/, nav2_core::GoalChecker * /*goal_checker*/)
{
  // resiti
  if (need_init_)
  {
    init_rotation(pose);
  }

  if ((ir_sensor_detected_ || cancelling_))
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

void Rotation::setPlan(const nav_msgs::msg::Path & path)
{
  // Intentionally implemented only for one pose goal. todo zemo
  target_pose_ = path.poses[0];
  need_init_ = true;
}


void Rotation::init_stopping(const double & speed)
{
  if (ruckig_ != nullptr)
  {
    delete ruckig_;
  }

  ruckig_ = new ruckig::Ruckig<1>{1.0 / update_rate_};
  ruckig_input_.max_velocity = {max_ang_vel_ * stopping_coeff_};
  ruckig_input_.max_acceleration = {max_ang_acc_ * stopping_coeff_};
  ruckig_input_.max_jerk = {max_ang_jerk_ * stopping_coeff_};
  ruckig_input_.target_velocity = {0.0};
  ruckig_input_.target_acceleration = {0.0};
  ruckig_input_.current_velocity = {speed};
  //ruckig_input_.current_acceleration = {0};
  ruckig_input_.control_interface = ruckig::ControlInterface::Velocity;

  state_ = STOPPING;
}

void Rotation::init_rotation(const geometry_msgs::msg::PoseStamped current_pose)
{
  double robot_yaw = tf2::getYaw(current_pose.pose.orientation);
  target_angle_ = tf2::getYaw(target_pose_.pose.orientation);

  // todo zemo hocemo li najkraci ili duzi ugao da radimo
  const double remaining = remaining_angle(current_pose);

  if (ir_sensor_detected_)
  {
    // vec na destinaciji ili uopste ne krecemo todo zemo
    // promeniti impelmentaciju controller servera kako bi ovo postigao
    throw nav2_core::ControllerException("Unable to init movement rotation!");
  }
  else if (std::fabs(remaining) < yaw_tolerance_)
  {
    /* code */
    RCLCPP_INFO(logger_, "No need for rotation!, already at target yaw!");
    return;
  }

  RCLCPP_INFO(logger_, "UGAO ROBOTA JE: %lf, ROTTAION ANGLE je = %lf, angle left = %lf", robot_yaw, target_angle_, remaining);

  RCLCPP_INFO(logger_, "REVERSING JE: %d, LONGER_ROUTE je = %d", reversing_, longer_route_);

  if (ruckig_ != nullptr)
  {
    delete ruckig_;
  }
  
  ruckig_ = new ruckig::Ruckig<1>{1.0 / update_rate_};
  ruckig_input_.max_velocity = {max_ang_vel_};
  ruckig_input_.max_acceleration = {max_ang_acc_};
  ruckig_input_.max_jerk = {max_ang_jerk_};
  ruckig_input_.target_position = {remaining};
  ruckig_input_.current_velocity = {0};
  ruckig_input_.current_acceleration = {0};
  ruckig_input_.current_position = {0};
  ruckig_input_.control_interface = ruckig::ControlInterface::Position;
  //ruckig_->update(ruckig_input_, ruckig_output_);

  state_ = ROTATING;
  need_init_ = false;
}

void Rotation::regulate_velocity(const geometry_msgs::msg::PoseStamped /*current_pose*/, geometry_msgs::msg::TwistStamped *cmd_vel)
{
  const bool is_trajectory_finished = (ruckig_->update(ruckig_input_, ruckig_output_) == ruckig::Finished);
  switch (state_)
  {
  case ROTATING:
    {
    // double remaining = remaining_angle(current_pose);
    double next_step = ruckig_output_.new_position[0] - ruckig_input_.current_position[0];
    //double next_step = remaining - ruckig_output_.new_position[0];

    //RCLCPP_INFO(logger_, "new: %lf, cur: %lf je ", ruckig_output_.new_position[0], ruckig_input_.current_position[0]);
    //RCLCPP_INFO(logger_, "next step: %lf, update_rate: %lf je XXX", next_step, update_rate_);
    
    // dx/dt (dt[s] = 1 / update rate[Hz])
    cmd_vel->twist.angular.z = next_step * update_rate_;
    // RCLCPP_INFO(logger_, "Nalazimo se u rotation i state je znak: %lf, a cmd_vel %lf, nest_step je %lf", angular_vel_sign_, cmd_vel->twist.angular.z, next_step);
    }
    break;
  case STOPPING:
    {
    cmd_vel->twist.linear.x = ruckig_output_.new_velocity[0];
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
  else if (cancelling_)
  {
    finished_cancelling_ = true;
  }
  else
  {
    throw nav2_core::ControllerException("Stopped becouse of detetction");
  }
}

double Rotation::remaining_angle(const geometry_msgs::msg::PoseStamped current_pose)
{
  double robot_yaw = tf2::getYaw(current_pose.pose.orientation);

  if (reversing_)
  {
    reverse_M_PI(&robot_yaw);
  }

  const double difference = target_angle_ - robot_yaw;

  double remaining_angle = std::atan2(std::sin(difference),std::cos(difference));

  if (longer_route_ )
  {
    reverse_2_M_PI(&remaining_angle);
  }

  return remaining_angle;
}

void Rotation::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
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

  ir_sensor_detected_ = false;
  need_init_ = true;
  reversing_ = false;
  longer_route_ = false;
  state_ = IDLE;

  //-------------------------------------------------
  max_ang_vel_ = 5.0;
  max_ang_acc_ = 10.0;
  max_ang_jerk_ = 100.0;
  //-------------------------------------------------

  // params
  node->get_parameter("min_theta_velocity_threshold", min_ang_vel_tolerance_);

  node->get_parameter("controller_frequency", update_rate_);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".stopping_coeff", rclcpp::ParameterValue(4.0));
  stopping_coeff_ = node->get_parameter(plugin_name_ + ".stopping_coeff").as_double();

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".yaw_tolerance", rclcpp::ParameterValue(0.01));
  yaw_tolerance_ = node->get_parameter(plugin_name_ + ".yaw_tolerance").as_double();
  // ---------------------------------

  turtle_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 1);
  
  ir_sensor_subscription_ = node->create_subscription<std_msgs::msg::Int32>(
    "/ir_sensor_state", 10, std::bind(&Rotation::ir_sensor_callback, this, std::placeholders::_1));
}

void Rotation::cleanup()
{
  turtle_pub_.reset();

  state_ = IDLE;

  // ----------------------
  global_path_pub_.reset();
  carrot_pub_.reset();
  curvature_carrot_pub_.reset();
  is_rotating_to_heading_pub_.reset();
  // --------------------
}

bool Rotation::cancel()
{
  // zalika od reset todo zemo
  if(state_ == ROTATING) 
  { 
    cancelling_ = true;
  }

  return finished_cancelling_;
}

void Rotation::setSpeedLimit(
  const double & /*speed_limit*/,
  const bool & /*percentage*/)
{
  // Intentionally unused in the minimal test implementation.
}

void Rotation::reset()
{
  //-----------------------------
  cancelling_ = false;
  finished_cancelling_ = false;
  // has_reached_xy_tolerance_ = false;
  //------------------------------
  ir_sensor_detected_ = false;
  reversing_ = false;
  longer_route_ = false;
  need_init_ = true;
  state_ = IDLE;
}

void Rotation::activate()
{
  // No publishers are created in this minimal test implementation.
  turtle_pub_->on_activate();
}

void Rotation::deactivate()
{
  // No publishers are created in this minimal test implementation.
  turtle_pub_->on_deactivate();
}

void Rotation::ir_sensor_callback(const std_msgs::msg::Int32::SharedPtr msg)
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

}  // namespace zemo_navigation

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(zemo_navigation::Rotation, nav2_core::Controller)
