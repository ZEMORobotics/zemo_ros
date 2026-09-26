#ifndef MOVE_HPP
#define MOVE_HPP

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_util/execution_timer.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "zemo_msgs/action/move_command.hpp"
#include "zemo_msgs/msg/move_properties.hpp"
#include "zemo_msgs/msg/move_state.hpp"
#include "std_msgs/msg/int32.hpp"
#include <ruckig/ruckig.hpp>
#include "rclcpp/rclcpp.hpp"

typedef zemo_msgs::action::MoveCommand move_command;
typedef zemo_msgs::msg::MoveState move_state;

namespace zemo_navigation
{
    class MoveActionServer : public rclcpp::Node
    {
        public:
            using GoalHandleMove = rclcpp_action::ServerGoalHandle<move_command>;

            explicit MoveActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
            //~MoveActionServer();

        private:

            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid , std::shared_ptr<const move_command::Goal> goal);
            rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle);
            void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle);

            void execute(const std::shared_ptr<GoalHandleMove> goal_handle);

            void update();
            void param_decleration();

            //void on_action();

            bool init_move();
            void init_rotation(double diff_yaw);
            void regulate_rotation(geometry_msgs::msg::TwistStamped *cmd_vel, double diff_yaw);
            void init_translation(double diff_x, double diff_y);
            void regulate_translation(geometry_msgs::msg::TwistStamped *cmd_vel, double diff_x, double diff_y);

            void state_rotating_towards_goal(const tf2::Transform &tf_base_target, const tf2::Transform &tf_odom_base, geometry_msgs::msg::TwistStamped *cmd_vel);
            void state_translating(const tf2::Transform &tf_base_target, geometry_msgs::msg::TwistStamped *cmd_vel);
            void state_rotating_at_goal(const tf2::Transform &tf_base_target, geometry_msgs::msg::TwistStamped *cmd_vel);
            void state_stopping();

            double get_diff_heading(const tf2::Transform &tf_base_target);
            double get_diff_final_orientation(const tf2::Transform &tf_base_target);
            double get_distance(const tf2::Transform &tf_base_target);

            void sensor_callback(const std_msgs::msg::Int32::SharedPtr msg);

            bool update_odom_target_tf();

            void instant_robot_stop();
            void update_state_msg(tf2::Transform &tf_base_target);

            int update_rate_;
            double stopping_distance_;

            std::shared_ptr<move_state> state_msg_;


            rclcpp_action::Server<move_command>::SharedPtr action_server_;

            rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
            rclcpp::Subscription<move_command>::SharedPtr command_sub_;
            rclcpp::Publisher<move_state>::SharedPtr state_pub_;

            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ir_sensor_subscription_;

            std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
            std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
            std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;

            std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
            std::unique_ptr<tf2_ros::Buffer> tf_;

            std::shared_ptr<move_command::Goal> command_;
            std::shared_ptr<move_command::Goal> default_command_{new move_command::Goal()};

            std::string robot_frame_;
            std::string odom_frame_;                        // da viidm sta cu sa ovim

            tf2::Transform tf_odom_target_;
            rclcpp::Time end_time_;
            bool target_updated_{false};
            double last_error_x_;
            double last_error_y_;
            double last_error_yaw_;

            rclcpp::Time debouncing_end_;
            rclcpp::Duration debouncing_duration_{0, 0};
            void debouncing_reset();

            double angular_stuck_coeff_;
            double linear_stuck_coeff_;
            bool is_sensor_detected_;
            bool reversing_;

            ruckig::Ruckig<1> *rotation_ruckig_{nullptr};
            ruckig::InputParameter<1> rotation_ruckig_input_;
            ruckig::OutputParameter<1> rotation_ruckig_output_;

            rclcpp::Time start_action_time_;

            tf2::Transform locked_tf_odom_base_;
            bool lock_tf_odom_base_{false};
            double lock_tf_odom_base_threshold_;

            ruckig::Ruckig<1> *translation_ruckig_{nullptr};
            ruckig::InputParameter<1> translation_ruckig_input_;
            ruckig::OutputParameter<1> translation_ruckig_output_;

            rclcpp::Duration command_timeout_{0, 0};
            rclcpp::Time last_command_received_;

            uint8_t state_{move_state::STATE_IDLE};
            uint8_t previous_state_{move_state::STATE_IDLE};

                    

    };
}

#endif