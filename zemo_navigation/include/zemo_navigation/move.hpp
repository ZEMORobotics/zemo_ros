#ifndef MOVE_HPP
#define MOVE_HPP

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"
//#include "nav2_costmap_2d/costmap_subscriber.hpp"
//#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "zemo_msgs/action/move_command.hpp"
#include "zemo_msgs/msg/move_properties.hpp"
#include "zemo_msgs/msg/move_debug.hpp"
#include "std_msgs/msg/int32.hpp"
#include <ruckig/ruckig.hpp>

// use_sim_time: false  obavezno

typedef zemo_msgs::action::MoveCommand MoveCommand;
typedef zemo_msgs::msg::MoveDebug MoveDebug;

typedef enum 
    {STATE_IDLE, STATE_ROTATING_TOWARDS_GOAL, STATE_TRANSLATING, STATE_ROTATING_AT_GOAL, STATE_STOPPING}
    MoveStates;

namespace zemo_navigation
{
    class MoveActionServer : public rclcpp::Node
    {
        public:
            using GoalHandleMove = rclcpp_action::ServerGoalHandle<MoveCommand>;

            explicit MoveActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
            //~MoveActionServer();

        private:

            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid , std::shared_ptr<const MoveCommand::Goal> goal);
            rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle);
            void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle);

            void execute(const std::shared_ptr<GoalHandleMove> goal_handle);

            void update();

            void nav2_init();
            bool init_move();
            void init_rotation(double remaining_yaw);
            void regulate_rotation(geometry_msgs::msg::TwistStamped *cmd_vel, double remaining_yaw);
            void init_translation(double remaining_x);
            void init_stopping();
            void regulate_translation(geometry_msgs::msg::TwistStamped *cmd_vel, double remaining_x, double remaining_y);

            void state_rotating_towards_goal(const tf2::Transform &tf_base_target, const tf2::Transform &tf_odom_base, geometry_msgs::msg::TwistStamped *cmd_vel);
            void state_translating(const tf2::Transform &tf_base_target, geometry_msgs::msg::TwistStamped *cmd_vel);
            void state_rotating_at_goal(const tf2::Transform &tf_base_target, geometry_msgs::msg::TwistStamped *cmd_vel);
            void state_stopping(const tf2::Transform &tf_base_target, geometry_msgs::msg::TwistStamped *cmd_vel);

            double get_remaining_heading(const tf2::Transform &tf_base_target);
            double get_remaining_final_orientation(const tf2::Transform &tf_base_target);
            double get_distance(const tf2::Transform &tf_base_target);

            bool update_odom_target_tf();

            void instant_robot_stop();

            void ir_sensor_callback(const std_msgs::msg::Int32::SharedPtr msg);             // TODO QOS

            void update_debug_msg(tf2::Transform &tf_base_target);                          // TODO Debug msg create
            std::shared_ptr<MoveDebug> debug_msg_;
            bool debug_node_active_;


            int update_rate_;
            double stopping_distance_;

            rclcpp_action::Server<MoveCommand>::SharedPtr action_server_;
            std::shared_ptr<GoalHandleMove> current_goal_;
            bool finished_;
            //bool goal_active_{false};
            //std::mutex goal_mutex_;

            rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
            rclcpp::Publisher<MoveDebug>::SharedPtr debug_pub_;
            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ir_sensor_subscription_;

            std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
            std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
            std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;

            std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
            std::unique_ptr<tf2_ros::Buffer> tf_;

            std::shared_ptr<MoveCommand::Goal> command_;
            std::shared_ptr<MoveCommand::Goal> default_command_{std::make_shared<MoveCommand::Goal>()};
            std::shared_ptr<MoveCommand::Result> result_{std::make_shared<MoveCommand::Result>()};
            std::shared_ptr<MoveCommand::Feedback> feedback_{std::make_shared<MoveCommand::Feedback>()};

            std::string robot_frame_;
            std::string odom_frame_;

            tf2::Transform tf_odom_target_;
            rclcpp::Time end_time_;
            //bool target_updated_{false};
            double next_step_x_;
            double next_step_y_;
            double next_step_yaw_;
            geometry_msgs::msg::TwistStamped last_cmd_vel_;

            rclcpp::Time debouncing_end_;
            rclcpp::Duration debouncing_duration_{0, 0};
            void debouncing_reset();

            bool is_sensor_detected_;
            bool reversing_;
            double reversing_coeff_;             // regulate je po x pa ne treba

            ruckig::Ruckig<1> *rotation_ruckig_{nullptr};
            ruckig::InputParameter<1> rotation_ruckig_input_;
            ruckig::OutputParameter<1> rotation_ruckig_output_;

            tf2::Transform locked_tf_odom_base_;
            bool lock_tf_odom_base_{false};
            double lock_tf_odom_base_threshold_;

            ruckig::Ruckig<1> *translation_ruckig_{nullptr};
            ruckig::InputParameter<1> translation_ruckig_input_;
            ruckig::OutputParameter<1> translation_ruckig_output_;
            double min_velocity_tolerance_;

            rclcpp::Duration command_timeout_{0, 0};
            //rclcpp::Time last_command_received_;

            MoveStates state_{STATE_IDLE};
            MoveStates previous_state_{STATE_IDLE};

                    

    };
}

#endif