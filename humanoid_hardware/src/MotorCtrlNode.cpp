#include "humanoid_hardware/MotorCtrlNode.hpp"

MotorControlNode::MotorControlNode() : Node("motor_control_node"), running_(true) {
    // Initialize CAN interface
    socket_fd_ = can_init("can0");
    if (socket_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN interface.");
        rclcpp::shutdown();
    }

    // Initialize motors and ROS 2 interfaces
    motor_feedback_pub_ = this->create_publisher<humanoid_interfaces::msg::MotorFeedback>("humanoid_interfaces/motor_feedback", 10);
    motor_command_sub_ = this->create_subscription<humanoid_interfaces::msg::MotorCommand>(
        "humanoid_interfaces/motor_commands", 10, 
        std::bind(&MotorControlNode::motor_command_callback, this, std::placeholders::_1)
    );

    // Initialize motors
    for (int i = 0; i < 10; ++i) {
        motors_.emplace_back();
        motors_[i].init("placeholder", 0x00 + i, 0x50 + i); // Assign command and feedback CAN IDs
    }

    // Start the receiving thread
    receiver_thread_ = std::thread(&MotorControlNode::can_receive, this);

    RCLCPP_INFO(this->get_logger(), "MotorControlNode started.");
}

MotorControlNode::~MotorControlNode() {
    running_ = false;  // Signal the receiving thread to stop
    if (receiver_thread_.joinable()) {
        receiver_thread_.join();  // Wait for the thread to finish
    }
    close(socket_fd_);  // Close CAN socket
}

int MotorControlNode::can_init(const char *interface) {
    int socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error while opening socket: %s", strerror(errno));
        return -1;
    }
    
    int buffer_size = 1024 * 1024;
    setsockopt(socket_fd, SOL_SOCKET, SO_SNDBUF, &buffer_size, sizeof(buffer_size));

    int enable_canfd = 1;
    if (setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error enabling CAN FD support: %s", strerror(errno));
        close(socket_fd);
        return -1;
    }

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, interface);
    if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error getting interface index: %s", strerror(errno));
        close(socket_fd);
        return -1;
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error in socket bind: %s", strerror(errno));
        close(socket_fd);
        return -1;
    }

    return socket_fd;
}

bool MotorControlNode::can_transmit(int can_id, const uint8_t *data, uint8_t data_len) {
    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = data_len;
    std::memcpy(frame.data, data, data_len);

    if (write(socket_fd_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error while sending CAN frame: %s", strerror(errno));
        return false;
    }
    return true;
}

void MotorControlNode::can_receive() {
    struct canfd_frame frame;
    while (rclcpp::ok() && running_) {
        int nbytes = read(socket_fd_, &frame, sizeof(struct canfd_frame));
        if (nbytes < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error reading CAN frame: %s", strerror(errno));
            break;
        }

        for (auto &motor : motors_) {
            if (frame.can_id == motor.getFeedbackCanId()) {
                motor.decode_sensor_feedback(frame.data);
                
                humanoid_interfaces::msg::MotorFeedback feedback_msg;
                feedback_msg.motor_name = motor.getName();
                const auto &feedback = motor.getSensorFeedback();
                feedback_msg.position = feedback.pos;
                feedback_msg.velocity = feedback.vel;
                feedback_msg.torque = feedback.torq;

                motor_feedback_pub_->publish(feedback_msg);
                RCLCPP_INFO(this->get_logger(), "Published feedback for motor %s", feedback_msg.motor_name.c_str());
            }
        }
    }
}

void MotorControlNode::motor_command_callback(const humanoid_interfaces::msg::MotorCommand::SharedPtr msg) {
    auto motor_it = std::find_if(motors_.begin(), motors_.end(), [&](DaMiaoMotor &motor) {
        return motor.getName() == msg->motor_name;
    });

    if (motor_it != motors_.end()) {
        motor_it->set_cmd(msg->position, msg->velocity, msg->torque, 1.0, 0.4);  // Example kp, kd values
        uint8_t cmd_data[8];
        motor_it->encode_cmd_msg(cmd_data);
        can_transmit(motor_it->getCmdCanId(), cmd_data, 8);
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}