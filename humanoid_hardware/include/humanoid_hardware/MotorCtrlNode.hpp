#ifndef MOTOR_CTRL_NODE_HPP
#define MOTOR_CTRL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <vector>
#include "DaMiaoMotor.hpp"
#include "humanoid_interfaces/msg/motor_command.hpp"
#include "humanoid_interfaces/msg/motor_feedback.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class MotorControlNode : public rclcpp::Node {
public:
    MotorControlNode();
    ~MotorControlNode();

private:
    // CAN initialization and communication functions
    int can_init(const char *interface);
    void can_receive();  // Receiving function for CAN messages
    bool can_transmit(int can_id, const uint8_t *data, uint8_t data_len);

    // Function to send motor commands
    void motor_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // ROS 2 Publishers and Subscribers
    rclcpp::Publisher<humanoid_interfaces::msg::MotorFeedback>::SharedPtr motor_feedback_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_command_sub_;

    // Thread management
    std::thread receiver_thread_;
    std::atomic<bool> running_;  // Flag to control the receiving thread

    // Member variables
    int socket_fd_;
    std::vector<DaMiaoMotor> motors_;
};

#endif // MOTOR_CTRL_NODE_HPP
