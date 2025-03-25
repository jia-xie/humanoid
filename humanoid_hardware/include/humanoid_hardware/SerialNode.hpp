#ifndef SERIAL_NODE_HPP
#define SERIAL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <array>
#include <string>
#include <humanoid_interfaces/msg/remote_signal.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"


class SerialNode : public rclcpp::Node {
public:
    SerialNode();
    ~SerialNode();

private:
    int serial_port_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<humanoid_interfaces::msg::RemoteSignal>::SharedPtr remote_signal_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr imu_pub_;
    void readDataCallback();
    bool setupSerialPort(const std::string &port, int baud_rate);
    std::chrono::steady_clock::time_point last_frame_time_;
    bool first_frame_received_ = false;
    int reading_period_us;
};

#endif // SERIAL_NODE_HPP
