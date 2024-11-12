#ifndef SERIAL_NODE_HPP
#define SERIAL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <array>
#include <string>

class SerialNode : public rclcpp::Node {
public:
    SerialNode();
    ~SerialNode();

private:
    int serial_port_;
    rclcpp::TimerBase::SharedPtr timer_;
    void readDataCallback();
    bool setupSerialPort(const std::string &port, int baud_rate);
    std::chrono::steady_clock::time_point last_frame_time_;
    bool first_frame_received_ = false;
    int reading_period_us;
};

#endif // SERIAL_NODE_HPP
