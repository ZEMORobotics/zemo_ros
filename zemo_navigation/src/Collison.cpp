
#include <chrono>
#include <memory>
#include <string>
#include <functional>
#include <exception>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"

using namespace std::chrono_literals;

class CollisionPublisher : public rclcpp::Node
{
public:
  CollisionPublisher()
  : Node("collision_publisher")
  {
    publisher_ =
      create_publisher<std_msgs::msg::String>("topic", 10);

    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());

    // The listener populates this node's TF buffer.
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_);
  }

  void nav2_init()
  {
    // Create subscribers only after the Node is fully constructed.
    costmap_sub_ =
      std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
        shared_from_this(), "local_costmap/costmap_raw");

    footprint_sub_ =
      std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
        shared_from_this(),
        "local_costmap/published_footprint",
        *tf_,
        "base_link");

    collision_checker_ =
      std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
        *costmap_sub_,
        *footprint_sub_,
        get_name());

    // Start the timer only after the checker exists.
    timer_ = create_wall_timer(
      500ms,
      std::bind(&CollisionPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    if (!collision_checker_) {
      RCLCPP_WARN(get_logger(), "Collision checker is not initialized");
      return;
    }

    // scorePose expects geometry_msgs::msg::Pose.
    geometry_msgs::msg::Pose2D pose;
    pose.x = 0.1;
    pose.y = 1.0;
    pose.theta = 0.0;


    auto message = std_msgs::msg::String();

    try {
      const double score = collision_checker_->scorePose(pose);

      if (score >= 254.0) {
        message.data = "COLLISION DETECTED!";
      } else {
        message.data = "NO COLLISION";
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(), "Collision check unavailable: %s", e.what());
      return;
    }

    RCLCPP_INFO(
      get_logger(), "Publishing: '%s'", message.data.c_str());

    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;

  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker>
    collision_checker_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CollisionPublisher>();

  node->nav2_init();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
