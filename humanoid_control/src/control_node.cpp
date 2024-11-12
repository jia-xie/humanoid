#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "humanoid_interfaces/msg/motor_feedback.hpp"
#include <unordered_map>
#include <mutex>

class ControlNode : public rclcpp::Node
{
public:
    ControlNode() : Node("control_node")
    {
        RCLCPP_INFO(this->get_logger(), "Starting Control Node");

        // Initialize motor_data_ with default motors and default values
        std::vector<std::string> motor_names = {
            "left_hip_pitch_joint", "left_knee_joint", "right_hip_pitch_joint", "right_knee_joint",
            "left_hip_yaw_joint", "left_hip_roll_joint", "right_hip_yaw_joint", "right_hip_roll_joint"};

        for (const auto &motor_name : motor_names)
        {
            motor_data_[motor_name] = {0.0, 0.0, 0.0}; // Default position, velocity, and effort
        }

        // Publisher for joint states (for visualization in RViz)
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // Publisher for motor commands using JointState message
        motor_command_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("humanoid_interfaces/motor_commands", 10);

        // Subscriber for motor feedback
        motor_feedback_subscriber_ = this->create_subscription<humanoid_interfaces::msg::MotorFeedback>(
            "humanoid_interfaces/motor_feedback", 10,
            std::bind(&ControlNode::motorFeedbackCallback, this, std::placeholders::_1));

        // Timer to publish joint states at 10 Hz
        publish_joint_state_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&ControlNode::publishJointStates, this));

        // Timer to publish motor commands at 10 Hz
        publish_motor_command_timer_ = this->create_wall_timer(
            std::chrono::microseconds(2000), // 10 Hz
            std::bind(&ControlNode::publishMotorCommands, this));
    }

private:
    void motorFeedbackCallback(const humanoid_interfaces::msg::MotorFeedback::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // Update the data structure with the latest motor feedback
        motor_data_[msg->motor_name] = {
            msg->position,
            msg->velocity,
            msg->torque};
    }

    void publishJointStates()
    {
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->get_clock()->now();

        // Lock the data to ensure thread safety
        std::lock_guard<std::mutex> lock(data_mutex_);

        // Fill the JointState message with names, positions, velocities, and efforts
        for (const auto &entry : motor_data_)
        {
            joint_state_msg.name.push_back(entry.first);
            joint_state_msg.position.push_back(entry.second.position);
            joint_state_msg.velocity.push_back(entry.second.velocity);
            joint_state_msg.effort.push_back(entry.second.effort);
        }

        // Publish the JointState message to /joint_states for visualization in RViz
        joint_state_publisher_->publish(joint_state_msg);
    }

    void publishMotorCommands()
    {
        auto motor_command_msg = sensor_msgs::msg::JointState();
        motor_command_msg.header.stamp = this->get_clock()->now();

        // Lock the data to ensure thread safety
        std::lock_guard<std::mutex> lock(data_mutex_);

        // Fill the JointState message with names, desired positions, velocities, and efforts (for control)
        for (const auto &entry : motor_data_)
        {
            motor_command_msg.name.push_back(entry.first);
            motor_command_msg.position.push_back(0.0);  // Example desired position
            motor_command_msg.velocity.push_back(0.0);  // Example desired velocity
            motor_command_msg.effort.push_back(0.0);    // Example desired torque
        }

        // Publish the JointState message as motor commands
        motor_command_publisher_->publish(motor_command_msg);
        RCLCPP_INFO(this->get_logger(), "Published motor commands for all joints.");
    }

    // Data structure to store the latest feedback for each motor
    struct MotorData
    {
        float position;
        float velocity;
        float effort;
    };
    std::unordered_map<std::string, MotorData> motor_data_;

    // ROS publishers and subscriber
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_command_publisher_;
    rclcpp::Subscription<humanoid_interfaces::msg::MotorFeedback>::SharedPtr motor_feedback_subscriber_;

    // Timers for publishing joint states and motor commands
    rclcpp::TimerBase::SharedPtr publish_joint_state_timer_;
    rclcpp::TimerBase::SharedPtr publish_motor_command_timer_;

    // Mutex for thread-safe access to motor data
    std::mutex data_mutex_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
