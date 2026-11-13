// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ZEMO_NAVIGATION_ROTATION_CONTROLLER_HPP_
#define ZEMO_NAVIGATION_ROTATION_CONTROLLER_HPP_

#include <string>
#include <memory>
#include <cmath>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/msg/collision_detector_state.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include <ruckig/ruckig.hpp>

typedef enum {IDLE, ROTATING, STOPPING} 
  MoveStates;

namespace zemo_navigation
{

/**
 * @class zemo_navigation::Rotation
 * @brief Regulated pure pursuit controller plugin
 */
class Rotation : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for zemo_navigation::Rotation
   */
  Rotation() = default;

  /**
   * @brief Destrructor for zemo_navigation::Rotation
   */
  ~Rotation() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

  /**
   * @brief Cancel the current control action with the controller
   * @return True if the controller has finished stopping and the action can be terminated,
   * false if the robot is still decelerating
   */
  bool cancel() override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /**
   * @brief Reset the controller state (cancel flags, sensor detection flags,
   * movement direction and state machine) so that a new plan can be started
   */
  void reset() override;

protected:

  // zemo source code
  /**
   * @brief Callback for the IR sensor topic, updates the IR detection flag
   * @param msg Int32 message, 1 if an obstacle is detected, 0 otherwise
   */
  void ir_sensor_callback(const std_msgs::msg::Int32::SharedPtr msg);

  /**
   * @brief Compute the remaining distance along the circular arc to the target pose
   * @param current_pose Current robot pose
   * @return Remaining arc length in meters
   */
  double remaining_distance(const geometry_msgs::msg::PoseStamped current_pose);
  double remaining_angle(const geometry_msgs::msg::PoseStamped current_pose);

  void init_rotation(const geometry_msgs::msg::PoseStamped current_pose);

  void init_translation(const geometry_msgs::msg::PoseStamped current_pose);

  /**
   * @brief Initialize the stopping trajectory from the current speed to zero
   * @param speed Current linear speed of the robot in m/s
   */
  void init_stopping(const double & speed);    // proemni u twist todo zemo


   /**
   * @brief Compute the velocity command by advancing the trajectory generator one control step using ruckig
   * @param current_pose Current robot pose
   * @param cmd_vel Pointer to the velocity command that is filled with linear and angular velocity
   */
  void regulate_velocity(const geometry_msgs::msg::PoseStamped current_pose, geometry_msgs::msg::TwistStamped *cmd_vel);
  
  // void regulate_translate_velocity(const geometry_msgs::msg::PoseStamped current_pose, geometry_msgs::msg::TwistStamped *cmd_vel);
  // void regulate_rotate_velocity(const geometry_msgs::msg::PoseStamped current_pose, geometry_msgs::msg::TwistStamped *cmd_vel);
  

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("Rotation")};
  rclcpp::Clock::SharedPtr clock_;

  //nav2_regulated_pure_pursuit_controller::Parameters * params_;
  // ---------------------------------------------------------------------------------
  double goal_dist_tol_;
  double control_duration_;
  bool cancelling_ = false;
  bool finished_cancelling_ = false;
  bool is_rotating_to_heading_ = false;
  bool has_reached_xy_tolerance_ = false;  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>> carrot_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>> curvature_carrot_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>> is_rotating_to_heading_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> carrot_arc_pub_;
  // ---------------------------------------------------------------------------------

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>> turtle_pub_; // delete todo zemo

  ruckig::Ruckig<1> *ruckig_{nullptr};
  ruckig::InputParameter<1> ruckig_input_;
  ruckig::OutputParameter<1> ruckig_output_;
        
  double update_rate_;
  bool need_init_;
            // pocetni put
  MoveStates state_;

  bool reversing_;
  bool longer_route_;             // param from goal  
  double angular_vel_sign_;
  double target_angle_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ir_sensor_subscription_;
  bool ir_sensor_detected_;

  // params
  double stopping_coeff_;                          
  double min_ang_vel_tolerance_;
  double yaw_tolerance_;

  double max_ang_vel_, max_ang_acc_, max_ang_jerk_;

  geometry_msgs::msg::PoseStamped target_pose_;

  //std::unique_ptr<nav2_regulated_pure_pursuit_controller::PathHandler> path_handler_;
  //std::unique_ptr<nav2_regulated_pure_pursuit_controller::ParameterHandler> param_handler_;
  //std::unique_ptr<nav2_regulated_pure_pursuit_controller::CollisionChecker> collision_checker_;
};

}  // namespace zemo_navigation

#endif  // zemo_navigation__REGULATED_PURE_PURSUIT_CONTROLLEr_HPP_