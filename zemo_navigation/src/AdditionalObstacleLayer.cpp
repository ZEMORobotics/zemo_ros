#include "zemo_navigation/AdditionalObstacleLayer.hpp"

namespace zemo_navigation
{
    AdditionalObstacleLayer::AdditionalObstacleLayer()
    {
        /*last_max_x_ = std::numeric_limits<double>::lowest();
        last_max_y_ = std::numeric_limits<double>::lowest();
        last_min_x_ = std::numeric_limits<double>::max();
        last_min_y_ = std::numeric_limits<double>::max();*/
        obstacles_ = std::make_shared<std::unordered_map<std::string, std::vector<geometry_msgs::msg::Point>>>();
        active_obstcles_ = std::make_shared<std::unordered_map<std::string, std::vector<geometry_msgs::msg::Point>>>();
        delete_obstcles_ = std::make_shared<std::vector<std::vector<geometry_msgs::msg::Point>>>();
    }
    
    void AdditionalObstacleLayer::onInitialize()
    {
        auto node = node_.lock();
        current_ = true;
        
        declareParameter("enabled", rclcpp::ParameterValue(true));
        node->get_parameter(name_ + "." + "enabled", enabled_);

        //declareParameter("global_frame", rclcpp::ParameterValue("map"));
        node->get_parameter("global_frame", global_frame_);
    
        declareParameter("add_obstacle_topic", rclcpp::ParameterValue("/add_obstacle"));
        node->get_parameter(name_ + "." + "add_obstacle_topic", add_obstacle_topic_);

        declareParameter("del_obstacle_topic", rclcpp::ParameterValue("/del_obstacle"));
        node->get_parameter(name_ + "." + "del_obstacle_topic", delete_obstacle_topic_);

        std::vector<std::string> obstacle_labels_;
        declareParameter("obstacle_labels", rclcpp::ParameterValue(std::vector<std::string>()));
        node->get_parameter(name_ + "." + "obstacle_labels", obstacle_labels_);

        for (const auto &label : obstacle_labels_)
        {
            std::string geomrtry_points_raw;
            std::string error;
            auto points = std::vector<geometry_msgs::msg::Point>();

            declareParameter(label, rclcpp::ParameterValue(std::string("")));
            node->get_parameter(name_ + "." + label, geomrtry_points_raw);
            
            auto point_pairs = std::vector<geometry_msgs::msg::Point>();
            std::vector<std::vector<float>> float_pairs = nav2_util::parseVVF(geomrtry_points_raw, error);
            for (auto pair : float_pairs)
            {
                geometry_msgs::msg::Point temp_point_pair;
                temp_point_pair.x = pair[0];
                temp_point_pair.y = pair[1];
                points.push_back(temp_point_pair);
            }
            
            obstacles_->insert({label, points});
        }
        
        new_obstacle_sub_ = node->create_subscription<std_msgs::msg::String>(
            add_obstacle_topic_,10 , std::bind(&AdditionalObstacleLayer::new_obstacle_callback, this, std::placeholders::_1));
        
        del_obstacle_sub_ = node->create_subscription<std_msgs::msg::String>(
            delete_obstacle_topic_,10 , std::bind(&AdditionalObstacleLayer::del_obstacle_callback, this, std::placeholders::_1));
    }

    void AdditionalObstacleLayer::new_obstacle_callback(const std_msgs::msg::String::SharedPtr obstacle_label)
    {
        auto obstcle_it = obstacles_->find(obstacle_label->data);

        if (obstcle_it != obstacles_->end()) 
        {
            active_obstcles_->insert({obstcle_it->first, obstcle_it->second});
        }
    }

    void AdditionalObstacleLayer::del_obstacle_callback(const std_msgs::msg::String::SharedPtr obstacle_label)
    {
        auto obstcle_it = active_obstcles_->find(obstacle_label->data);

        if (obstcle_it != obstacles_->end()) 
        {
            delete_obstcles_->push_back(obstcle_it->second);
            active_obstcles_->erase(obstcle_it);
        }
    }

    void AdditionalObstacleLayer::updateBounds(
            double robot_x, double robot_y, double robot_yaw, double * min_x,
            double * min_y, double * max_x, double * max_y)
    {
        (void)robot_x;
        (void)robot_y;
        (void)robot_yaw;

        for (auto obstacle : *active_obstcles_)
        {
            for (auto point : obstacle.second)
            {
                *min_x = std::min(point.x, *min_x);
                *min_y = std::min(point.y, *min_y);
                *max_x = std::max(point.x, *max_x);
                *max_y = std::max(point.y, *max_y);
            }   
        }
        for (auto obstacle : *delete_obstcles_)
        {
            for (auto point : obstacle)
            {
                *min_x = std::min(point.x, *min_x);
                *min_y = std::min(point.y, *min_y);
                *max_x = std::max(point.x, *max_x);
                *max_y = std::max(point.y, *max_y);
            }   
        }
        
    }

    void AdditionalObstacleLayer::updateCosts(
            nav2_costmap_2d::Costmap2D & master_grid,
            int min_i, int min_j, int max_i, int max_j)
    {
        (void)min_i;
        (void)min_j;
        (void)max_i;
        (void)max_j;

        if (!enabled_)
        {
            return;
        }

        for (auto polygon : *active_obstcles_)
        {
            master_grid.setConvexPolygonCost(polygon.second, nav2_costmap_2d::LETHAL_OBSTACLE);
        }

        for (auto polygon : *delete_obstcles_)
        {
            master_grid.setConvexPolygonCost(polygon, nav2_costmap_2d::FREE_SPACE);
        }

        delete_obstcles_->clear();
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(zemo_navigation::AdditionalObstacleLayer, nav2_costmap_2d::Layer)