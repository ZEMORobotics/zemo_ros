// Minimal CircularArcController implementation for testing plugin loading.
// This version intentionally does not implement navigation control.
// It returns zero velocity and avoids Regulated Pure Pursuit dependencies.

#include <memory>
#include <string>
#include <utility>

#include "zemo_navigation/controller_plugins/circular_arc.hpp"
#include "nav2_core/controller_exceptions.hpp"

#include "nav2_util/node_utils.hpp"

#define sign(x) (((x) > 0) - ((x) < 0))

static constexpr double PI_2 = M_PI * 2.0;

namespace zemo_navigation
{
inline static void angle_reverse(double* angle)
{    
  *angle >= 0 ? *angle-= M_PI : *angle+= M_PI;
}

void CircularArcController::shortest_path_direction(const double alpha, double robot_yaw)
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
  
  if (angle > M_PI_2 || angle < -M_PI_2)
  {
    reversing_ = true;
  }
}

geometry_msgs::msg::TwistStamped CircularArcController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*speed*/, nav2_core::GoalChecker * /*goal_checker*/)
{
  if (need_init_)
  {
    init_movement(pose);
  }

  // TODO puna formula za stopping distance
  //const double stopping_distance = (stopping_distance_ + ((speed.linear.x * speed.linear.x) / (2 * max_lin_acc_))
    //+ (std::fabs(speed.linear.x) / update_rate_) + ((std::fabs(speed.linear.x) * max_lin_acc_) / (2 * max_lin_jerk_)) * linear_vel_sign_);
  double stopping_distance = stopping_distance_ + ((ruckig_output_.new_velocity[0] * ruckig_output_.new_velocity[0]) / (2 * max_lin_acc_))
    + (std::fabs(ruckig_output_.new_velocity[0]) / update_rate_) + ((std::fabs(ruckig_output_.new_velocity[0]) * max_lin_acc_) / (2 * max_lin_jerk_));
  // for dynamic polygon implementation negative values a for backwords movement
  RCLCPP_INFO(logger_,"STOPPING DISTANCE:%lf, IZRACUNAT STOPPING:%lf", stopping_distance_, stopping_distance);
  stopping_distance*=linear_vel_sign_;
  stopping_distance_publish(stopping_distance);

  if (state_ == MOVING && (collison_detetected_ || ir_sensor_detected_ || cancelling_))
  {
    //stopping function
    init_stopping(ruckig_output_.new_velocity[0]);
  }

  // Intentionally stop the robot; this is only a plugin-loading/build test.
  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();

  regulate_velocity(pose, cmd_vel.get());
  
  return *cmd_vel;
}

void CircularArcController::setPlan(const nav_msgs::msg::Path & path)
{
  // Intentionally implemented only for one pose goal. todo
  target_pose_ = path.poses[0];
  need_init_ = true;
}


void CircularArcController::init_stopping(const double & speed)
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

  //ruckig_->update(ruckig_input_, ruckig_output_);

  state_ = STOPPING;
}

void CircularArcController::init_movement(const geometry_msgs::msg::PoseStamped current_pose)
{
  double dx = target_pose_.pose.position.x - current_pose.pose.position.x;
  double dy = target_pose_.pose.position.y - current_pose.pose.position.y;

  const double alpha = std::atan2(dy, dx);
  double robot_yaw = tf2::getYaw(current_pose.pose.orientation);

  shortest_path_direction(alpha, robot_yaw);
  if (reversing_)
  {
    angle_reverse(&robot_yaw);
    linear_vel_sign_ = -1.0;
  }
  else
  {
    linear_vel_sign_ = 1.0;
  }

  // base link transform
  const double gx = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  const double gy = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

  const double L2 = gx * gx + gy * gy;

  if (L2 < 1e-6 || ir_sensor_detected_ || collison_detetected_)
  {
    // vec na destinaciji ili uopste ne krecemo todo zemo
    return;
  }

  // r_ is signed and carries angular velocity direction
  r_ = L2 / (2.0 * gy);

  //const double theta1 =  2.0 * std::asin(L / (2.0 * r_));
  const double theta =  2.0 * std::atan2(gy, gx);
  //RCLCPP_INFO(logger_, "THETA1: %lf, THETA2: %lf je XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", theta1, theta);

  destination_lenght_ = std::fabs(theta * r_);

  RCLCPP_INFO(logger_, "UGAO ROBOTA JE: %lf, L = %lf, R = %lf, duzina polukruga = %lf, znak lin  je:%lf", robot_yaw, L2, r_, destination_lenght_, linear_vel_sign_);

  if (ruckig_ != nullptr)
  {
    delete ruckig_;
  }
  
  ruckig_ = new ruckig::Ruckig<1>{1.0 / update_rate_};
  ruckig_input_.max_velocity = {max_lin_vel_};
  ruckig_input_.max_acceleration = {max_lin_acc_};
  ruckig_input_.max_jerk = {max_lin_jerk_};
  ruckig_input_.target_position = {destination_lenght_};
  ruckig_input_.current_velocity = {0};
  ruckig_input_.current_acceleration = {0};
  ruckig_input_.current_position = {0};
  ruckig_input_.control_interface = ruckig::ControlInterface::Position;

  //ruckig_->update(ruckig_input_, ruckig_output_);

  state_ = MOVING;
  need_init_ = false;
}

void CircularArcController::regulate_velocity(const geometry_msgs::msg::PoseStamped /*current_pose*/, geometry_msgs::msg::TwistStamped *cmd_vel)
{
  //double remaining = remaining_distance(current_pose);
  const bool is_trajectory_finished = (ruckig_->update(ruckig_input_, ruckig_output_) == ruckig::Finished);
  double next_step = ruckig_output_.new_position[0] - ruckig_input_.current_position[0];
  //double next_step = remaining - ruckig_output_.new_position[0];

  //RCLCPP_INFO(logger_, "new: %lf, cur: %lf je ", ruckig_output_.new_position[0], ruckig_input_.current_position[0]);
  //RCLCPP_INFO(logger_, "next step: %lf, update_rate: %lf je XXX", next_step, update_rate_);
  
  // dx/dt (dt[s] = 1 / update rate[Hz])
  cmd_vel->twist.linear.x = next_step * update_rate_;
  cmd_vel->twist.angular.z = cmd_vel->twist.linear.x / r_;
  // sign addition r_ carries angulra sign so parametar is not needed
  cmd_vel->twist.linear.x *= linear_vel_sign_;

  cmd_vel->header.stamp = clock_->now();
  cmd_vel->header.frame_id = "base_link";

  if (!is_trajectory_finished)
  {
    ruckig_output_.pass_to_input(ruckig_input_);
  }
  // u slucaju da smo u stopping stanju ovako proveravmo kraj 
  else if (state_ == STOPPING)
  {
    finished_cancelling_ = true;
    reset();
  }
}

double CircularArcController::remaining_distance(const geometry_msgs::msg::PoseStamped current_pose)
{
  // racunamo da se robot krece po kruznici sto znaci da se R ne menja
  const double dx = target_pose_.pose.position.x - current_pose.pose.position.x;
  const double dy = target_pose_.pose.position.y - current_pose.pose.position.y;
  double robot_yaw = tf2::getYaw(current_pose.pose.orientation);

  if (reversing_)
  {
    angle_reverse(&robot_yaw);
  }

  const double gx = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  const double gy = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

  const double theta =  2.0 * std::atan2(gy, gx);

  return std::fabs(theta * r_);
}

void CircularArcController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
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
  // ---------------------------------

  stop_dist_pub_ = node->create_publisher<std_msgs::msg::Float32>(collison_disatnce_topic_, 1);    // zemo todo zemo qs

  collison_detector_subscription_ = node->create_subscription<nav2_msgs::msg::CollisionDetectorState>(
    "/collision_detector_state", 10, std::bind(&CircularArcController::collision_detector_callback, this, std::placeholders::_1));
  
  ir_sensor_subscription_ = node->create_subscription<std_msgs::msg::Int32>(
    "/ir_sensor_state", 10, std::bind(&CircularArcController::ir_sensor_callback, this, std::placeholders::_1));
}

void CircularArcController::cleanup()
{
  stop_dist_pub_.reset();

  state_ = IDLE;

  // ----------------------
  global_path_pub_.reset();
  carrot_pub_.reset();
  curvature_carrot_pub_.reset();
  is_rotating_to_heading_pub_.reset();
  // --------------------
}

bool CircularArcController::cancel()
{
  // zalika od reset todo zemo
  if(state_ == MOVING) 
  { 
    cancelling_ = true;
  }

  return finished_cancelling_;
}

void CircularArcController::setSpeedLimit(
  const double & /*speed_limit*/,
  const bool & /*percentage*/)
{
  // Intentionally unused in the minimal test implementation.
}

void CircularArcController::reset()
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

void CircularArcController::activate()
{
  // No publishers are created in this minimal test implementation.
  stop_dist_pub_->on_activate();
}

void CircularArcController::deactivate()
{
  // No publishers are created in this minimal test implementation.
  stop_dist_pub_->on_deactivate();
}

void CircularArcController::stopping_distance_publish(const double stopping_distance)
{
  if (stop_dist_pub_->is_activated())
  {
    std_msgs::msg::Float32 msg;
    msg.data = stopping_distance;
    stop_dist_pub_->publish(msg);
  }
}

void CircularArcController::ir_sensor_callback(const std_msgs::msg::Int32::SharedPtr msg)
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

void CircularArcController::collision_detector_callback(const nav2_msgs::msg::CollisionDetectorState::SharedPtr msg)
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
PLUGINLIB_EXPORT_CLASS(zemo_navigation::CircularArcController, nav2_core::Controller)
