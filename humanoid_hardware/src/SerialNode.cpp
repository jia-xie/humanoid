#include "humanoid_hardware/SerialNode.hpp"

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <iomanip>

SerialNode::SerialNode() : Node("serial_node")
{
    // Configure the serial port
    std::string port = "/dev/ttyTHS0"; // Adjust port as needed
    int baud_rate = B1152000;          // Match the STM32 baud rate

    if (!setupSerialPort(port, baud_rate))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open or configure the serial port.");
        return;
    }

    // Start reading data periodically
    reading_period_us = 100; // stm32 sends at 1000Hz
    timer_ = this->create_wall_timer(
        std::chrono::microseconds(reading_period_us),
        std::bind(&SerialNode::readDataCallback, this));
    remote_signal_pub_ = this->create_publisher<humanoid_interfaces::msg::RemoteSignal>("/remote_signal", 10);
    imu_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/imu", 10);
}

SerialNode::~SerialNode()
{
    if (serial_port_ != -1)
    {
        close(serial_port_);
    }
}

bool SerialNode::setupSerialPort(const std::string &port, int baud_rate)
{
    // Open the serial port in read/write mode, no controlling terminal, and no delay
    serial_port_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (serial_port_ == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Error %i from open: %s", errno, strerror(errno));
        return false;
    }
    else
    {
        tcflush(serial_port_, TCIOFLUSH); // Flush both input and output buffers
    }

    // Configure serial port settings
    struct termios tty;
    if (tcgetattr(serial_port_, &tty) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error %i from tcgetattr: %s", errno, strerror(errno));
        close(serial_port_);
        return false;
    }

    // Set baud rate
    if (cfsetispeed(&tty, baud_rate) != 0 || cfsetospeed(&tty, baud_rate) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error setting baud rate: %s", strerror(errno));
        close(serial_port_);
        return false;
    }

    // Set 8N1 (8 data bits, no parity, 1 stop bit)
    tty.c_cflag &= ~PARENB; // Disable parity
    tty.c_cflag &= ~CSTOPB; // Use one stop bit
    tty.c_cflag &= ~CSIZE;  // Clear current data size setting
    tty.c_cflag |= CS8;     // 8 data bits

    // Disable hardware flow control
    tty.c_cflag &= ~CRTSCTS;

    // Enable reading and ignore modem control lines
    tty.c_cflag |= CREAD | CLOCAL;

    // Set raw mode (disable canonical mode, echo, signals, etc.)
    cfmakeraw(&tty);

    // Set blocking mode with a 1-second read timeout
    tty.c_cc[VMIN] = 1;   // Minimum 1 character to read
    tty.c_cc[VTIME] = 10; // Timeout in deciseconds (1 second)

    // Apply the configuration to the serial port
    if (tcsetattr(serial_port_, TCSANOW, &tty) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s", errno, strerror(errno));
        close(serial_port_);
        return false;
    }

    return true;
}

void SerialNode::readDataCallback()
{
    static int timeout_count = 0;
    static int error_print_count = 0; // used for not printing error every iteration
    if (!rclcpp::ok())
    {
        return; // Exit if ROS is shutting down
    }
    typedef struct
    {
        uint8_t header;
        float gyro[3];
        float accel[3];
        float rad[3]; // roll pitch yaw
        uint8_t remote_buffer[18];
    } DecodedData;
    uint8_t buffer[sizeof(DecodedData)];
    int num_bytes = read(serial_port_, buffer, sizeof(buffer));
    // Print the raw message in hexadecimal format
    std::stringstream hex_stream;
    if (num_bytes < 0)
    {
        timeout_count++;
        if (timeout_count * reading_period_us > 1000 * 10) // if timeout for 10 ms
        {

            timeout_count = 0; // clear timeout_count
            error_print_count++;
            if (error_print_count >= 200) // print error message every 2 seconds
            {
                RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", strerror(errno));

                error_print_count = 0;
            }
        }

        return;
    }
    timeout_count = 0; // clear timeout_count
    error_print_count = 0;

    // Calculate time since last frame
    // auto current_time = std::chrono::steady_clock::now();
    // if (first_frame_received_)
    // {
    //     auto time_diff = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_frame_time_).count();
    //     RCLCPP_INFO(this->get_logger(), "Time between frames: %ld us", time_diff);
    // }
    // else
    // {
    //     first_frame_received_ = true; // Skip calculation for the first frame
    // }
    // Update last frame time
    // last_frame_time_ = current_time;

    if (num_bytes != 45)
    {
        RCLCPP_DEBUG(this->get_logger(), "Received %d bytes, expected 45", num_bytes);
        return;
    }

    // Check header byte
    if (buffer[0] != 0xAA)
    {
        RCLCPP_WARN(this->get_logger(), "Invalid header byte");
        return;
    }

    DecodedData decoded_data;
    decoded_data.header = buffer[0];
    if (decoded_data.header != 0xAA)
    {
        // Handle invalid header error if needed
        return;
    }

    // Extract gyroscope data (3 floats)
    memcpy(&decoded_data.gyro[0], &buffer[1], sizeof(float));
    memcpy(&decoded_data.gyro[1], &buffer[1 + 1 * sizeof(float)], sizeof(float));
    memcpy(&decoded_data.gyro[2], &buffer[1 + 2 * sizeof(float)], sizeof(float));

    // Extract accelerometer data (3 floats)
    memcpy(&decoded_data.accel[0], &buffer[1 + 3 * sizeof(float)], sizeof(float));
    memcpy(&decoded_data.accel[1], &buffer[1 + 4 * sizeof(float)], sizeof(float));
    memcpy(&decoded_data.accel[2], &buffer[1 + 5 * sizeof(float)], sizeof(float));

    // Extract radian data (3 flaots)
    // memcpy(&tx_buffer[33], &g_imu.rad.roll, sizeof(float));
    // memcpy(&tx_buffer[33 + sizeof(float)], &g_imu.rad.pitch, sizeof(float));
    // memcpy(&tx_buffer[33 + sizeof(float) * 2], &g_imu.rad.yaw, sizeof(float));
    memcpy(&decoded_data.rad[0], &buffer[33], sizeof(float));
    memcpy(&decoded_data.rad[1], &buffer[33 + sizeof(float)], sizeof(float));
    memcpy(&decoded_data.rad[2], &buffer[33 + sizeof(float) * 2], sizeof(float));

    decoded_data.rad[0] += M_PI;
    if (decoded_data.rad[0] > M_PI)
    {
        decoded_data.rad[0] -= 2 * M_PI;
    }
    float tmp[3];
    tmp[0] = -decoded_data.rad[1];
    tmp[1] = decoded_data.rad[0];
    tmp[2] = decoded_data.rad[2];

    float veltmp[3];
    veltmp[0] = decoded_data.gyro[1];
    veltmp[1] = decoded_data.gyro[0];
    veltmp[2] = -decoded_data.gyro[2];

    std_msgs::msg::Float64MultiArray msg;
    msg.data.insert(msg.data.end(), veltmp, veltmp + 3);
    msg.data.insert(msg.data.end(), decoded_data.accel, decoded_data.accel + 3);
    msg.data.insert(msg.data.end(), tmp, tmp + 3);

    imu_pub_->publish(msg);

    uint8_t remote_buffer[8];
    memcpy(remote_buffer, buffer + 25, 8);

    // Decode and Publish the remote signal message
    auto remote_signal_msg = humanoid_interfaces::msg::RemoteSignal();
    // remote_signal_msg.header.stamp = this->get_clock()->now();
    remote_signal_msg.right_stick_x = (((remote_buffer[0] | (remote_buffer[1] << 8)) & 0x07ff) - 1024) / 660.0f;
    remote_signal_msg.right_stick_y = ((((remote_buffer[1] >> 3) | (remote_buffer[2] << 5)) & 0x07ff) - 1024) / 660.0f;
    remote_signal_msg.left_stick_x = ((((remote_buffer[2] >> 6) | (remote_buffer[3] << 2) | (remote_buffer[4] << 10)) & 0x07ff) - 1024) / 660.0f;
    remote_signal_msg.left_stick_y = ((((remote_buffer[4] >> 1) | (remote_buffer[5] << 7)) & 0x07ff) - 1024) / 660.0f;
    remote_signal_msg.wheel = ((remote_buffer[6] | (remote_buffer[7] << 8)) & 0x07FF) - 1024;
    remote_signal_msg.left_switch = ((remote_buffer[5] >> 4) & 0x000C) >> 2;
    remote_signal_msg.right_switch = ((remote_buffer[5] >> 4) & 0x0003);
    remote_signal_msg.online = (remote_buffer[7] & 0b00010000) >> 4;
    remote_signal_pub_->publish(remote_signal_msg);
    static int print_serial_count = 0;
    print_serial_count++;
    if (print_serial_count >= 5000)
    {
        print_serial_count = 0;
        RCLCPP_DEBUG(this->get_logger(), "%s", hex_stream.str().c_str());
        int16_t right_stick_x = ((remote_buffer[0] | (remote_buffer[1] << 8)) & 0x07ff) - 1024;
        int16_t right_stick_y = (((remote_buffer[1] >> 3) | (remote_buffer[2] << 5)) & 0x07ff) - 1024;
        int16_t left_stick_x = (((remote_buffer[2] >> 6) | (remote_buffer[3] << 2) | (remote_buffer[4] << 10)) & 0x07ff) - 1024;
        int16_t left_stick_y = (((remote_buffer[4] >> 1) | (remote_buffer[5] << 7)) & 0x07ff) - 1024;
        int16_t wheel = ((remote_buffer[6] | (remote_buffer[7] << 8)) & 0x07FF) - 1024;
        int16_t left_switch = ((remote_buffer[5] >> 4) & 0x000C) >> 2;
        int16_t right_switch = ((remote_buffer[5] >> 4) & 0x0003);
        uint8_t online = (remote_buffer[7] & 0b00010000) >> 4;

        RCLCPP_DEBUG(this->get_logger(), "[x: %d, y: %d], r: %d, [x: %d, y: %d], l: %d, w: %d, gyro: [x: %3f, y: %3f, z: %3f], accel: [x: %3f, y: %3f, z: %3f], rad: [roll: %f, pitch: %f, yaw: %f], online: %d",
                    right_stick_x, right_stick_y, right_switch, left_stick_x, left_stick_y, left_switch, wheel,
                    decoded_data.gyro[0], decoded_data.gyro[1], decoded_data.gyro[2], decoded_data.accel[0], decoded_data.accel[1], decoded_data.accel[2],
                    decoded_data.rad[0], decoded_data.rad[1], decoded_data.rad[2], online);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
