#ifndef ADITIONAL_OBSTACLE_LAYER_HPP
#define ADITIONAL_OBSTACLE_LAYER_HPP

//#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/array_parser.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "geometry_msgs/msg/polygon.hpp"
#include "std_msgs/msg/string.hpp"
#include <unordered_map>
#include <vector>
#include <string>

namespace zemo_navigation
{
    class AdditionalObstacleLayer : public nav2_costmap_2d::Layer
    {
    public:
        AdditionalObstacleLayer();
        
        virtual void onInitialize();
        virtual void updateBounds(
            double robot_x, double robot_y, double robot_yaw, double * min_x,
            double * min_y, double * max_x, double * max_y);
        virtual void updateCosts(
            nav2_costmap_2d::Costmap2D & master_grid,
            int min_i, int min_j, int max_i, int max_j);

        virtual void reset() { return; }
        virtual bool isClearable() { return false; }

        void new_obstacle_callback(const std_msgs::msg::String::SharedPtr obstacle_lable);
        void del_obstacle_callback(const std_msgs::msg::String::SharedPtr obstacle_lable);

    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr new_obstacle_sub_; 
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr del_obstacle_sub_; 

        std::shared_ptr<std::unordered_map<std::string, std::vector<geometry_msgs::msg::Point>>> obstacles_;
        std::shared_ptr<std::unordered_map<std::string, std::vector<geometry_msgs::msg::Point>>> active_obstcles_;
        std::shared_ptr<std::vector<std::vector<geometry_msgs::msg::Point>>> delete_obstcles_;
        
        //double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
        std::string global_frame_;
        std::string add_obstacle_topic_;
        std::string delete_obstacle_topic_;

    };  // class AdditionalObstacleLayer
}   // namespace zemo_navigation

#endif