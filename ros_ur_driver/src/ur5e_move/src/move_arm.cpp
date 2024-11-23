#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <vector>
#include <chrono>

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
        joint_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "ur_manipulator");
    }

 private:
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        auto callback_time = this->now();  // Timestamp when callback is triggered
        RCLCPP_INFO(this->get_logger(), "[%f] Entered joint callback...", callback_time.seconds());

        if (msg->position.size() >= 6) {
            joint_now = {msg->position[0], msg->position[1], msg->position[2],
                         msg->position[3], msg->position[4], msg->position[5]};

            RCLCPP_INFO(this->get_logger(), "[%f] Setting joint values...", this->now().seconds());
            bool success = joint_group_->setJointValueTarget(joint_now);
            if (success) {
                RCLCPP_INFO(this->get_logger(), "[%f] Position reached... Good job!!", this->now().seconds());
            } else {
                RCLCPP_WARN(this->get_logger(), "[%f] Failed to reach the position.", this->now().seconds());
            }

            // Record the planning start time
            auto planning_start = this->now();
            RCLCPP_INFO(this->get_logger(), "[%f] Starting motion planning...", planning_start.seconds());

            moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
            bool plan_success = (joint_group_->plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            
            // Record the planning end time
            auto planning_end = this->now();
            RCLCPP_INFO(this->get_logger(), "[%f] Planning completed.", planning_end.seconds());

            // Calculate and log planning duration
            double planning_duration = (planning_end - planning_start).seconds();
            RCLCPP_INFO(this->get_logger(), "Planning Time: %f seconds", planning_duration);

            if (plan_success) {
                // Execute and log the time taken for execution
                auto execution_start = this->now();
                RCLCPP_INFO(this->get_logger(), "[%f] Starting motion execution...", execution_start.seconds());
                
                joint_group_->move();
                
                auto execution_end = this->now();
                double execution_duration = (execution_end - execution_start).seconds();
                RCLCPP_INFO(this->get_logger(), "[%f] Execution completed. Execution Time: %f seconds", execution_end.seconds(), execution_duration);
            } else {
                RCLCPP_WARN(this->get_logger(), "Motion planning failed.");
            }

            joint_prev = joint_now;

            RCLCPP_INFO(this->get_logger(), "Updated joint positions: [%f, %f, %f, %f, %f, %f]",
                        joint_now[0], joint_now[1], joint_now[2],
                        joint_now[3], joint_now[4], joint_now[5]);
        } else {
            RCLCPP_WARN(this->get_logger(), "Received joint_state message with insufficient positions");
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

