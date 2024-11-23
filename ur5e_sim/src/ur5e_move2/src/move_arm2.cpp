#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <vector>
#include "rcl/time.h"
#include "rclcpp/clock.hpp"
#include<chrono>

using std::placeholders::_1;

bool ready_to_plan = true;
std::vector<double> joint_prev{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<double> joint_now{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

class ArmJoint : public rclcpp::Node {
 public:
    ArmJoint(const rclcpp::Node::SharedPtr& node)
        : Node("joint_sub"), node_(node) {
        // Create subscription to joint states
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/ur5e/joint_states", 10,
            std::bind(&ArmJoint::joint_callback, this, std::placeholders::_1));

        // Initialize the MoveGroupInterface outside of the constructor
        joint_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm_joints");
        //joint_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "ur_manipulator");
    }

 private:
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Calculate and log the latency of subscribing to the topic in nanoseconds
        //auto current_time = this->now().nanoseconds();
        auto msg_time = msg->header.stamp.nanosec;
        //auto latency = current_time - msg_time;
        // Assume this->now().nanoseconds() gives the current time in nanoseconds since the epoch.
        auto now = std::chrono::system_clock::now();

        // Convert to a time_t to manipulate the date and time components
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm local_tm = *std::localtime(&now_c);

        // Set the time to the start of today (midnight)
        local_tm.tm_hour = 0;
        local_tm.tm_min = 0;
        local_tm.tm_sec = 0;

        // Convert back to a time_point
        auto start_of_today = std::chrono::system_clock::from_time_t(std::mktime(&local_tm));

        // Calculate the difference in milliseconds
        auto milliseconds_today = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_of_today).count();
        // Convert milliseconds_today to nanoseconds for consistent unit comparison
        auto nanoseconds_today = milliseconds_today * 1'000'000;
        // Log latency in nanoseconds
        RCLCPP_INFO(this->get_logger(), "Subscription latency for /ur5e/joint_states: %ld milliseconds",milliseconds_today - msg_time);
        //RCLCPP_INFO(this->get_logger(), "Subscription latency__ for /ur5e/joint_states: %ld nanoseconds", msg_time);

        RCLCPP_INFO(this->get_logger(), "Entered joint callback...");
        
        // Check if message contains sufficient joint positions
        if (msg->position.size() >= 6) {
            joint_now = {msg->position[0], msg->position[1], msg->position[2],
                         msg->position[3], msg->position[4], msg->position[5]};

            RCLCPP_INFO(this->get_logger(), "Setting joint values...");
            bool success = joint_group_->setJointValueTarget(joint_now);
            if (success) {
                RCLCPP_INFO(this->get_logger(), "Position reached successfully.");
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to reach the desired position.");
            }

            // Measure planning time in nanoseconds
            auto plan_start_time = this->now();
            moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
            bool plan_success = joint_group_->plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
            auto plan_end_time = this->now();
            rclcpp::Duration plan_duration = plan_end_time - plan_start_time;
            RCLCPP_INFO(this->get_logger(), "Planning time: %ld nanoseconds", plan_duration.nanoseconds());

            // Measure execution time only if planning was successful
            if (plan_success) {
                auto exec_start_time = this->now();
                joint_group_->move();
                auto exec_end_time = this->now();
                rclcpp::Duration exec_duration = exec_end_time - exec_start_time;
                RCLCPP_INFO(this->get_logger(), "Execution time: %ld nanoseconds", exec_duration.nanoseconds());
            } else {
                RCLCPP_WARN(this->get_logger(), "Planning failed; skipping execution.");
            }

            // Update previous joint positions
            joint_prev = joint_now;

            // Log updated joint positions
            RCLCPP_INFO(this->get_logger(), "Updated joint positions: [%f, %f, %f, %f, %f, %f]",
                        joint_now[0], joint_now[1], joint_now[2],
                        joint_now[3], joint_now[4], joint_now[5]);
        } else {
            RCLCPP_WARN(this->get_logger(), "Received joint_state message with insufficient positions.");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> joint_group_;
    rclcpp::Node::SharedPtr node_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("move_arm_node");
    auto arm_joint_node = std::make_shared<ArmJoint>(node);
    rclcpp::spin(arm_joint_node);
    rclcpp::shutdown();
    return 0;
}
