#include "rclcpp/rclcpp.hpp"
#include "humanoid_interfaces/msg/motor_feedback.hpp"
#include "sensor_msgs/msg/joint_state.hpp" // this is for rviz (publisher variable joint_state_pub_)
#include "humanoid_interfaces/msg/motor_command.hpp"

class MotorAbstractionNode : public rclcpp::Node
{
public:
    MotorAbstractionNode();
    ~MotorAbstractionNode();

private:
    void allocate_motor_data();
    void publish_rviz_joint_states(); //publish joint states for rviz2 visualization
    void process_motor_stats(humanoid_interfaces::msg::MotorFeedback::SharedPtr motor_feedback);
    void dispatch_motor_commands();
    rclcpp::Subscription<humanoid_interfaces::msg::MotorFeedback>::SharedPtr motor_feedback_sub_; // this subscriber will get motor feedback data from motor control node
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_command_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rviz_joint_state_pub_;

    std::vector<std::string> motor_names;
    // Data structure to store the latest data (feedback from motor and command that will be sent out)
    struct MotorData
    {
        float position;
        float velocity;
        float torque;
    };
    std::unordered_map<std::string, MotorData> motor_stats_;
    std::mutex motor_stats_mutex_;
    std::unordered_map<std::string, MotorData> motor_commands_;
    std::mutex motor_commands_mutex_;

    // Timers
    rclcpp::TimerBase::SharedPtr publish_rviz_joint_state_timer_;
    rclcpp::TimerBase::SharedPtr dispatch_motor_commands_timer_;
};