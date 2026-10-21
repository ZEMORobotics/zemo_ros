#include "zemo_navigation/dynamic_polygon.hpp"

#include <math.h>
#include <cmath>
#include <exception>

#include "nav2_util/node_utils.hpp"

namespace zemo_navigation
{

DynamicPolygon::DynamicPolygon(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & polygon_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance)
: Polygon::Polygon(node, polygon_name, tf_buffer, base_frame_id, transform_tolerance)
{
  RCLCPP_INFO(logger_, "[%s]: Creating DynamicPolygon", polygon_name_.c_str());
}

DynamicPolygon::~DynamicPolygon()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying DynamicPolygon", polygon_name_.c_str());
}

bool DynamicPolygon::getParameters(
  std::string & polygon_sub_topic,
  std::string & polygon_pub_topic,
  std::string & footprint_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Clear the polygon subscription topic. It will be set later.
  polygon_sub_topic.clear();

  try 
  {
    // Leave it not initialized: the will cause an error if it will not set
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".wheel_separation", rclcpp::PARAMETER_DOUBLE);
    wheel_separation_ = node->get_parameter(polygon_name_ + ".wheel_separation").as_double();
  } catch (const rclcpp::exceptions::ParameterUninitializedException &) 
  {
    RCLCPP_INFO(logger_, "[%s]: Wheel separation is not defined. Using static value instead.", polygon_name_.c_str());
      wheel_separation_ = 0.5;
  }

  // dynamic polygon init
  poly_.assign({{0.0, wheel_separation_/2}, {wheel_separation_, wheel_separation_/2}, {wheel_separation_, -(wheel_separation_/2)}, {0.0, -(wheel_separation_/2)}});

  bool ret = true;
  if (!getCommonParameters(
      polygon_sub_topic, polygon_pub_topic, footprint_topic, true))
  {
    if (polygon_sub_topic.empty()) {
      RCLCPP_ERROR(
        logger_,
        "[%s]: Error while getting DynamicPolygon parameters: static stopping distamce and sub topic both not defined",
        polygon_name_.c_str());
    }
    ret = false;
  }

  // There is no footprint subscription for the DynamicPolygon. Thus, set string as empty.
  footprint_topic.clear();

  return ret;
}

void DynamicPolygon::createSubscription(std::string & polygon_sub_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!polygon_sub_topic.empty()) {
    RCLCPP_INFO(
      logger_,
      "[%s]: Subscribing on %s topic for polygon",
      polygon_name_.c_str(), polygon_sub_topic.c_str());
    rclcpp::QoS polygon_qos = rclcpp::SystemDefaultsQoS();  // set to default
    if (polygon_subscribe_transient_local_) {
      polygon_qos.transient_local();
    }
    stopping_dis_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      polygon_sub_topic, polygon_qos,
      std::bind(&DynamicPolygon::stopDisatnceCallback, this, std::placeholders::_1));
  }
}

void DynamicPolygon::updatePolygon(double stopping_distance)
{
  // Update DynamicPolygon radius
  stopping_distance_ = stopping_distance;

  // Create a polygon from radius and store it
  std::vector<nav2_collision_monitor::Point> poly;
  getPolygon(poly);
  polygon_.polygon.points.clear();  // clear polygon points
  poly_.clear();
  for (nav2_collision_monitor::Point & p : poly) {
    if (p.x != 0.0)
    {
      p.x = stopping_distance_;
    }
    else
    {
      p.x = 0.0;
    }

    poly_.push_back({p.x, p.y});

    if(visualize_)
    {
      geometry_msgs::msg::Point32 p_s;
      p_s.x = p.x;
      p_s.y = p.y;
      RCLCPP_INFO(logger_, "[%f], [%f]: Points values", p_s.x, p_s.y);
      // p_s.z will remain 0.0
      polygon_.polygon.points.push_back(p_s);  // add new points
    }
  }
}

void DynamicPolygon::stopDisatnceCallback(std_msgs::msg::Float32::ConstSharedPtr msg)
{
  RCLCPP_INFO(
    logger_,
    "[%s]: Polygon DynamicPolygon radius update has been arrived",
    polygon_name_.c_str());
  updatePolygon(msg->data);
}

}  // namespace zemo_navigation