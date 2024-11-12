#include "humanoid_hardware/MotorCtrlNode.hpp"
#include <iomanip> 

MotorControlNode::MotorControlNode() : Node("motor_control_node"), running_(true)
{
    // Initialize CAN interface
    socket_fd_ = can_init("can0");
    if (socket_fd_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN interface.");
        rclcpp::shutdown();
    }

    // Load command and feedback CAN IDs, kp, and kd values
    std::vector<std::string> motor_names = {
            "left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
            "left_knee_joint", "left_ankle_joint",
            "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint",
            "right_knee_joint", "right_ankle_joint"
        };

    for (const auto &motor : motor_names)
    {
        this->declare_parameter<int>("cmd_can_id." + motor, 0);
        this->declare_parameter<int>("feedback_can_id." + motor, 0);
        this->declare_parameter<double>("kp." + motor, 0.0);
        this->declare_parameter<double>("kd." + motor, 0.0);

        int cmd_can_id, feedback_can_id;
        float kp, kd;

        this->get_parameter("cmd_can_id." + motor, cmd_can_id);
        this->get_parameter("feedback_can_id." + motor, feedback_can_id);
        this->get_parameter("kp." + motor, kp);
        this->get_parameter("kd." + motor, kd);

        motors_.emplace_back(motor, cmd_can_id, feedback_can_id, kp, kd);
        RCLCPP_INFO(this->get_logger(), "Loaded parameters for motor %s, cmd_can_id: %d, feedback_can_id: %d, kp: %f, kd: %f",
                    motor.c_str(), cmd_can_id, feedback_can_id, kp, kd);
    }

    // Initialize motors and ROS 2 interfaces
    motor_feedback_pub_ = this->create_publisher<humanoid_interfaces::msg::MotorFeedback>("humanoid_interfaces/motor_feedback", 10);
    
    // Subscribe to JointState messages for all motor commands
    motor_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "humanoid_interfaces/motor_commands", 10,
        std::bind(&MotorControlNode::motor_command_callback, this, std::placeholders::_1));
    

    // Start CAN receiver thread
    receiver_thread_ = std::thread(&MotorControlNode::can_receive, this);
    RCLCPP_INFO(this->get_logger(), "CAN receiving thread started.");

    RCLCPP_INFO(this->get_logger(), "MotorControlNode started.");
}

MotorControlNode::~MotorControlNode()
{
    running_ = false; // Signal the receiving thread to stop
    if (receiver_thread_.joinable())
    {
        receiver_thread_.join(); // Wait for the thread to finish
    }
    close(socket_fd_); // Close CAN socket
}

int MotorControlNode::can_init(const char *interface)
{
    int socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error while opening socket: %s", strerror(errno));
        return -1;
    }

    int buffer_size = 1024 * 1024;
    if (setsockopt(socket_fd, SOL_SOCKET, SO_SNDBUF, &buffer_size, sizeof(buffer_size)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set socket buffer size: %s", strerror(errno));
        close(socket_fd);
        return -1;
    }

    int enable_canfd = 1;
    if (setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error enabling CAN FD support: %s", strerror(errno));
        close(socket_fd);
        return -1;
    }

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, interface);

    // Attempt to get the interface index
    if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error getting interface index for %s: %s", interface, strerror(errno));
        close(socket_fd);
        return -1;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Interface %s found with index %d", interface, ifr.ifr_ifindex);
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // Attempt to bind the socket to the CAN interface
    if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error binding socket to interface %s: %s", interface, strerror(errno));
        close(socket_fd);
        return -1;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Successfully bound socket to interface %s", interface);
    }

    RCLCPP_INFO(this->get_logger(), "CAN initialized on %s with socket_fd %d", interface, socket_fd);
    return socket_fd;
}

bool MotorControlNode::can_transmit(int can_id, const uint8_t *data, uint8_t data_len)
{
    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = data_len;
    std::memcpy(frame.data, data, data_len);

    if (write(socket_fd_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "Error while sending CAN frame: %s", strerror(errno));
        return false;
    }
    return true;
}

void MotorControlNode::can_receive()
{
    struct canfd_frame frame;
    while (rclcpp::ok() && running_)
    {
        int nbytes = read(socket_fd_, &frame, sizeof(struct canfd_frame));
        if (nbytes < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error reading CAN frame: %s", strerror(errno));
            break;
        }
        else
        {
            // RCLCPP_INFO(this->get_logger(), "Received CAN frame - ID: 0x%X, DLC: %u", frame.can_id, frame.len);

            // Find the motor associated with the received feedback CAN ID
            auto motor_it = std::find_if(motors_.begin(), motors_.end(), [&](DaMiaoMotor &motor)
                                         { return motor.getFeedbackCanId() == frame.can_id; });
            
            if (motor_it != motors_.end())
            {
                // RCLCPP_INFO(this->get_logger(), "Processing feedback for motor: %s", motor_it->getName().c_str());

                // Decode the received sensor feedback data
                motor_it->decode_sensor_feedback(frame.data);

                // Create and populate the MotorFeedback message
                humanoid_interfaces::msg::MotorFeedback feedback_msg;
                feedback_msg.motor_name = motor_it->getName();
                const auto &feedback = motor_it->getSensorFeedback();
                feedback_msg.position = feedback.pos;
                feedback_msg.velocity = feedback.vel;
                feedback_msg.torque = feedback.torq;

                // Publish the feedback
                motor_feedback_pub_->publish(feedback_msg);
                // RCLCPP_INFO(this->get_logger(), "Published feedback for motor %s", feedback_msg.motor_name.c_str());
            }
            else
            {
                // RCLCPP_WARN(this->get_logger(), "Received CAN frame with unknown ID: 0x%X", frame.can_id);
            }
        }
    }
}



void MotorControlNode::motor_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received joint commands for all motors");

    // Ensure all arrays (name, position, velocity, effort) have the same length
    if (msg->name.size() != msg->position.size() || msg->name.size() != msg->velocity.size() || msg->name.size() != msg->effort.size()) {
        RCLCPP_ERROR(this->get_logger(), "JointState message arrays are of unequal length");
        return;
    }

    // Process each motor command in the JointState message
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        const auto &motor_name = msg->name[i];
        auto motor_it = std::find_if(motors_.begin(), motors_.end(), [&](DaMiaoMotor &motor)
                                     { return motor.getName() == motor_name; });
        
        if (motor_it != motors_.end())
        {
            // RCLCPP_INFO(this->get_logger(), "Found motor: %s", motor_it->getName().c_str());
            motor_it->set_cmd(msg->position[i], msg->velocity[i], msg->effort[i], 0.0, 0.0); // Example kp, kd values

            uint8_t cmd_data[8];
            motor_it->encode_cmd_msg(cmd_data);

            // RCLCPP_INFO(this->get_logger(), "Encoded command data for transmission: [%02X %02X %02X %02X %02X %02X %02X %02X]",
            //             cmd_data[0], cmd_data[1], cmd_data[2], cmd_data[3], cmd_data[4], cmd_data[5], cmd_data[6], cmd_data[7]);

            if (can_transmit(motor_it->getCmdCanId(), cmd_data, 8))
            {
                // RCLCPP_INFO(this->get_logger(), "CAN transmission successful for motor: %s, Cmd CAN ID: %d", motor_it->getName().c_str(), motor_it->getCmdCanId());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "CAN transmission failed for motor: %s, Cmd CAN ID: %d", motor_it->getName().c_str(), motor_it->getCmdCanId());
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Motor name %s not found in the list of motors.", motor_name.c_str());
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}
