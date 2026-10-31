#ifndef NAV2_COLLISION_MONITOR__DynamicPolygon_HPP_
#define NAV2_COLLISION_MONITOR__DynamicPolygon_HPP_

#include <memory>
#include <vector>
#include <string>

#include "std_msgs/msg/float32.hpp"

#include "nav2_collision_monitor/polygon.hpp"

namespace zemo_navigation
{

/**
 * @brief DynamicPolygon shape implementaiton.
 * For STOP/SLOWDOWN/LIMIT model it represents zone around the robot
 * while for APPROACH model it represents robot footprint.
 */
class DynamicPolygon : public nav2_collision_monitor::Polygon
{
public:
  /**
   * @brief DynamicPolygon class constructor
   * @param node Collision Monitor node pointer
   * @param polygon_name Name of DynamicPolygon
   * @param tf_buffer Shared pointer to a TF buffer
   * @param base_frame_id Robot base frame ID
   * @param transform_tolerance Transform tolerance
   */
  DynamicPolygon(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & polygon_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance);
  /**
   * @brief DynamicPolygon class destructor
   */
  ~DynamicPolygon();

protected:
  /**
   * @brief Supporting routine obtaining polygon-specific ROS-parameters
   * @param polygon_sub_topic Input name of polygon subscription topic
   * @param polygon_pub_topic Output name of polygon or radius publishing topic
   * @param footprint_topic Output name of footprint topic. For DynamicPolygon returns empty string,
   * there is no footprint subscription in this class.
   * @return True if all parameters were obtained or false in failure case
   */
  bool getParameters(
    std::string & polygon_sub_topic,
    std::string & polygon_pub_topic,
    std::string & footprint_topic) override;

  /**
   * @brief Creates polygon or radius topic subscription
   * @param polygon_sub_topic Output name of polygon or radius subscription topic.
   * Empty, if no polygon subscription.
   */
  void createSubscription(std::string & polygon_sub_topic) override;

  /**
   * @brief Updates polygon from radius value
   * @param radius New DynamicPolygon radius to update polygon
   */
  void updatePolygon(double stopping_distance);

  /**
   * @brief Dynamic DynamicPolygon radius callback
   * @param msg Shared pointer to the radius value message
   */
  void stopDisatnceCallback(std_msgs::msg::Float32::ConstSharedPtr msg);


  // ----- Variables -----

  /// @brief Stopping distance calculated in nav2 controller plugin
  double stopping_distance_;
  /// @brief Wheel separation of the robot that defines y cordinates for the base_link polygon
  double wheel_separation_;
  /// @brief Stopping distance subscription
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr stopping_dis_sub_;
};  // class DynamicPolygon

}  // namespace zemo_navigation

#endif  // NAV2_COLLISION_MONITOR__DynamicPolygon_HPP_