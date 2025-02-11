#include "humanoid_hardware/MotorAbstractionNode.hpp"

MotorAbstractionNode::MotorAbstractionNode() : Node("motor_abstraction_node")
{
    allocate_motor_data(); // initialization

    // create subscribers and publishers
    motor_feedback_sub_ = this->create_subscription<humanoid_interfaces::msg::MotorFeedback>(
        "humanoid_interfaces/motor_feedback", 10,
        std::bind(&MotorAbstractionNode::process_motor_stats, this, std::placeholders::_1));
    motor_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("humanoid_interfaces/motor_commands", 10);
    rviz_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // create a timers
    publish_rviz_joint_state_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&MotorAbstractionNode::publish_rviz_joint_states, this));
    dispatch_motor_commands_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&MotorAbstractionNode::dispatch_motor_commands, this));
}

MotorAbstractionNode::~MotorAbstractionNode()
{
}

/**
 * Initialized the data structure for storing all motor stats and pending commands
 */
void MotorAbstractionNode::allocate_motor_data()
{
    // Initialize motor_data_ with default motors and default values
    motor_names = {
        "left_hip_pitch_joint", "left_knee_joint", "right_hip_pitch_joint", "right_knee_joint",
        "left_hip_yaw_joint", "left_hip_roll_joint", "right_hip_yaw_joint", "right_hip_roll_joint"};

    for (const auto &motor_name : motor_names)
    {
        motor_stats_[motor_name] = {0.0, 0.0, 0.0}; // Default position, velocity, and effort
        motor_commands_[motor_name] = {0.0, 0.0, 0.0}; // Default position, velocity, and effort
    }
}

/**
 * Process the raw motor feedback data from the motor control node and process it such that matches 
 * URDF description. The stored data aligns with rviz2 visualization
 * 
 * @param motor_feedback The raw motor feedback data from the motor control node, it contains the original
 * information from the motor communication protocol (CAN bus)
 */
void MotorAbstractionNode::process_motor_stats(humanoid_interfaces::msg::MotorFeedback::SharedPtr motor_feedback)
{
    std::lock_guard<std::mutex> lock(motor_stats_mutex_);
    // Identify if the current feedback is from knee motor then calculate the knee angle with four bar mechanism
    if (motor_feedback->motor_name == "left_knee_joint")
    {
        motor_stats_["left_knee_joint"].position = motor_feedback->position + motor_stats_["left_hip_pitch_joint"].position;
    }
    else if (motor_feedback->motor_name == "right_knee_joint")
    {
        motor_stats_["right_knee_joint"].position = motor_feedback->position + motor_stats_["right_hip_pitch_joint"].position;
    }
    else
    {
        motor_stats_[motor_feedback->motor_name].position = motor_feedback->position;
    }
    // velocity and torque are directly copied
    motor_stats_[motor_feedback->motor_name].velocity = motor_feedback->velocity;
    motor_stats_[motor_feedback->motor_name].torque = motor_feedback->torque;
}

/**
 * This function publishes the motor commands to the motor control node
 */
void MotorAbstractionNode::dispatch_motor_commands()
{
    auto motor_command_msg = sensor_msgs::msg::JointState();
    motor_command_msg.header.stamp = this->get_clock()->now();

    std::lock_guard<std::mutex> lock(motor_commands_mutex_);

    for (const auto &motor_names : motor_names)
    {
        motor_command_msg.name.push_back(motor_names);
        motor_command_msg.position.push_back(motor_commands_[motor_names].position);
        motor_command_msg.velocity.push_back(motor_commands_[motor_names].velocity);
        motor_command_msg.effort.push_back(motor_commands_[motor_names].torque);
    }

    motor_command_pub_->publish(motor_command_msg);
}

    // void publishMotorCommands()
    // {
    //     auto motor_command_msg = sensor_msgs::msg::JointState();
    //     motor_command_msg.header.stamp = this->get_clock()->now();

    //     // Lock the data to ensure thread safety
    //     std::lock_guard<std::mutex> lock(data_mutex_);

    //     // Fill the JointState message with names, desired positions, velocities, and efforts (for control)
    //     for (const auto &entry : motor_data_)
    //     {
    //         motor_command_msg.name.push_back(entry.first);
    //         motor_command_msg.position.push_back(0.0);  // Example desired position
    //         motor_command_msg.velocity.push_back(0.0);  // Example desired velocity
    //         motor_command_msg.effort.push_back(0.0);    // Example desired torque
    //     }

    //     // Publish the JointState message as motor commands
    //     motor_command_publisher_->publish(motor_command_msg);
    //     // RCLCPP_INFO(this->get_logger(), "Published motor commands for all joints.");
    // }

/**
 * This is the callback function of wall timer to publish the joint states to rviz2 for visualization
 */
void MotorAbstractionNode::publish_rviz_joint_states()
{
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = this->get_clock()->now();

    std::lock_guard<std::mutex> lock(motor_stats_mutex_);

    for (const auto &entry : motor_stats_)
    {
        joint_state_msg.name.push_back(entry.first);
        joint_state_msg.position.push_back(entry.second.position);
        joint_state_msg.velocity.push_back(entry.second.velocity);
        joint_state_msg.effort.push_back(entry.second.torque);
    }

    rviz_joint_state_pub_->publish(joint_state_msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorAbstractionNode>());
    rclcpp::shutdown();
    return 0;
}