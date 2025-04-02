#include "humanoid_hardware/HumanoidSystemHardware.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace humanoid_hardware {

hardware_interface::CallbackReturn HumanoidSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  num_joints_ = info_.joints.size();
  joint_names_.reserve(num_joints_);

  hw_position_.resize(num_joints_, 0.0);
  hw_velocity_.resize(num_joints_, 0.0);
  hw_effort_.resize(num_joints_, 0.0);

  hw_cmd_position_.resize(num_joints_, 0.0);
  hw_cmd_velocity_.resize(num_joints_, 0.0);
  hw_cmd_effort_.resize(num_joints_, 0.0);
  hw_cmd_kp_.resize(num_joints_, 0.0);
  hw_cmd_kd_.resize(num_joints_, 0.0);

  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HumanoidSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < num_joints_; ++i) {
    state_interfaces.emplace_back(joint_names_[i], "position", &hw_position_[i]);
    state_interfaces.emplace_back(joint_names_[i], "velocity", &hw_velocity_[i]);
    state_interfaces.emplace_back(joint_names_[i], "effort", &hw_effort_[i]);
  }

  // IMU state
  state_interfaces.emplace_back("base_imu", "angular_velocity.x", &imu_ang_vel_[0]);
  state_interfaces.emplace_back("base_imu", "angular_velocity.y", &imu_ang_vel_[1]);
  state_interfaces.emplace_back("base_imu", "angular_velocity.z", &imu_ang_vel_[2]);

  state_interfaces.emplace_back("base_imu", "linear_acceleration.x", &imu_lin_acc_[0]);
  state_interfaces.emplace_back("base_imu", "linear_acceleration.y", &imu_lin_acc_[1]);
  state_interfaces.emplace_back("base_imu", "linear_acceleration.z", &imu_lin_acc_[2]);

  state_interfaces.emplace_back("base_imu", "orientation.x", &imu_orientation_[0]);
  state_interfaces.emplace_back("base_imu", "orientation.y", &imu_orientation_[1]);
  state_interfaces.emplace_back("base_imu", "orientation.z", &imu_orientation_[2]);
  state_interfaces.emplace_back("base_imu", "orientation.w", &imu_orientation_[3]);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HumanoidSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < num_joints_; ++i) {
    command_interfaces.emplace_back(joint_names_[i], "position", &hw_cmd_position_[i]);
    command_interfaces.emplace_back(joint_names_[i], "velocity", &hw_cmd_velocity_[i]);
    command_interfaces.emplace_back(joint_names_[i], "effort", &hw_cmd_effort_[i]);
    command_interfaces.emplace_back(joint_names_[i], "kp", &hw_cmd_kp_[i]);
    command_interfaces.emplace_back(joint_names_[i], "kd", &hw_cmd_kd_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type HumanoidSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // 🔧 Replace these with your real hardware read functions
  // Simulate sensor values or call DaMiaoMotor::get_state()
  for (size_t i = 0; i < num_joints_; ++i) {
    hw_position_[i] += 0.001;  // dummy increment
    hw_velocity_[i] = 0.1;
    hw_effort_[i] = 0.05;
  }

  // Fill IMU from hardware or dummy
  imu_ang_vel_[0] = 0.01;
  imu_ang_vel_[1] = 0.02;
  imu_ang_vel_[2] = 0.03;

  imu_lin_acc_[0] = 0.0;
  imu_lin_acc_[1] = 0.0;
  imu_lin_acc_[2] = 9.81;

  imu_orientation_[0] = 0.0;
  imu_orientation_[1] = 0.0;
  imu_orientation_[2] = 0.0;
  imu_orientation_[3] = 1.0;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HumanoidSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // 🔧 Replace this with your real hardware write (e.g., DaMiaoMotor::set_cmd())
  for (size_t i = 0; i < num_joints_; ++i) {
    RCLCPP_DEBUG(rclcpp::get_logger("HumanoidSystemHardware"),
      "Sending joint[%s]: pos=%.3f vel=%.3f eff=%.3f kp=%.3f kd=%.3f",
      joint_names_[i].c_str(), hw_cmd_position_[i], hw_cmd_velocity_[i],
      hw_cmd_effort_[i], hw_cmd_kp_[i], hw_cmd_kd_[i]);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace humanoid_hardware

PLUGINLIB_EXPORT_CLASS(humanoid_hardware::HumanoidSystemHardware, hardware_interface::SystemInterface)
