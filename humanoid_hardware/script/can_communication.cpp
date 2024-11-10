#include <iostream>
#include <cstring>
#include <cerrno>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <unistd.h>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <cmath>
#include <vector>
#include "DaMiaoMotor.hpp"

// Initialize CAN FD interface
int can_init(const char *interface)
{
    int socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0)
    {
        std::cerr << "Error while opening socket: " << strerror(errno) << std::endl;
        return -1;
    }
    int buffer_size = 1024 * 1024;
    setsockopt(socket_fd, SOL_SOCKET, SO_SNDBUF, &buffer_size, sizeof(buffer_size));

    int enable_canfd = 1;
    if (setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0)
    {
        std::cerr << "Error enabling CAN FD support: " << strerror(errno) << std::endl;
        close(socket_fd);
        return -1;
    }

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, interface);
    if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0)
    {
        std::cerr << "Error getting interface index: " << strerror(errno) << std::endl;
        close(socket_fd);
        return -1;
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        std::cerr << "Error in socket bind: " << strerror(errno) << std::endl;
        close(socket_fd);
        return -1;
    }

    return socket_fd;
}

// Transmit CAN message
bool can_transmit(int socket_fd, int can_id, const uint8_t *data, uint8_t data_len)
{
    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = data_len;
    std::memcpy(frame.data, data, data_len);

    if (write(socket_fd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        std::cerr << "Error while sending CAN frame: " << strerror(errno) << std::endl;
        return false;
    }
    // struct canfd_frame frame;
    // frame.can_id = can_id;
    // frame.len = data_len;  // Data length for CAN FD
    // frame.flags = CANFD_BRS;  // Enable Bit Rate Switch for faster data phase, if needed
    // std::memcpy(frame.data, data, data_len);


    // // Send the CAN FD frame
    // if (write(socket_fd, &frame, sizeof(struct canfd_frame)) != sizeof(struct canfd_frame)) {
    //     std::cerr << "Error sending CAN FD frame: " << strerror(errno) << std::endl;
    // } else {
    //     // std::cout << "CAN FD frame sent successfully." << std::endl;
    // }
    return true;
}

// Receive and decode CAN FD messages
void can_receive(int socket_fd, std::vector<DaMiaoMotor> &motors)
{
    struct canfd_frame frame;
    while (true)
    {
        int nbytes = read(socket_fd, &frame, sizeof(struct canfd_frame));
        if (nbytes < 0)
        {
            std::cerr << "Error reading CAN frame: " << strerror(errno) << std::endl;
            break;
        }

        for (auto &motor : motors)
        {
            if (frame.can_id == motor.getFeedbackCanId())
            {
                motor.decode_sensor_feedback(frame.data);
                const auto &feedback = motor.getSensorFeedback();
                std::cout << "Motor CAN ID: " << frame.can_id << " | Position: " << feedback.pos << std::endl;
            }
        }
    }
}

int main()
{
    // Initialize CAN interface (e.g., "can0") with 1000000 bps
    int socket_fd = can_init("can0");
    if (socket_fd < 0)
    {
        std::cerr << "Failed to initialize CAN interface." << std::endl;
        return -1;
    }

    // Initialize motors
    std::vector<DaMiaoMotor> motors(8);
    for (int i = 0; i < 8; ++i)
    {
        motors[i].init(0x00 + i, 0x50 + i); // Assign command and feedback CAN IDs
    }

    // Start the receive thread
    std::thread receiver_thread(can_receive, socket_fd, std::ref(motors));

    float initial_vel = 0.0, initial_torque = 0.0, kp = 1.0, kd = 0.4;

    // Define the fixed time step (2 ms for 500 Hz)
    std::chrono::microseconds time_step(1500); // 2 ms
    // Start time
    auto start_time = std::chrono::steady_clock::now();

    // Send motor command at 500 Hz with sinusoidal position
    while (true)
    {
        // Calculate the elapsed time since the start of the loop
        auto current_time = std::chrono::steady_clock::now();
        float t = std::chrono::duration<float>(current_time - start_time).count();

        // Calculate the position as a sinusoidal function with period 2 seconds
        float target_position = 3 * std::sin(0.2 * M_PI * t);

        // Loop through each motor to send commands
        for (auto &motor : motors)
        {
            // Set motor command with sinusoidal position
            motor.set_cmd(target_position, initial_vel, initial_torque, kp, kd);

            // Encode command message for transmission
            uint8_t cmd_data[8];
            motor.encode_cmd_msg(cmd_data);

            // Transmit motor command over CAN
            if (!can_transmit(socket_fd, motor.getCmdCanId(), cmd_data, 8))
            {
                std::cerr << "Failed to transmit motor command." << std::endl;
                close(socket_fd);
                return -1;
            }
        }

        // Calculate the time taken for this iteration
        auto end_time = std::chrono::steady_clock::now();
        auto iteration_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - current_time);

        // Calculate the remaining time to wait to maintain a 500 Hz frequency
        if (iteration_time < time_step)
        {
            std::this_thread::sleep_for(time_step - iteration_time);
        }
    }

    // Wait for the receiver thread to finish (optional, if there's a mechanism to stop it)
    receiver_thread.join();

    // Close sockets
    close(socket_fd);
    return 0;
}