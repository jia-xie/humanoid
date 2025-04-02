#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"

#include <vector>
#include <string>

namespace humanoid_hardware {

class HumanoidSystemHardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HumanoidSystemHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  size_t num_joints_;
  std::vector<std::string> joint_names_;

  std::vector<double> hw_position_;
  std::vector<double> hw_velocity_;
  std::vector<double> hw_effort_;

  std::vector<double> hw_cmd_position_;
  std::vector<double> hw_cmd_velocity_;
  std::vector<double> hw_cmd_effort_;
  std::vector<double> hw_cmd_kp_;
  std::vector<double> hw_cmd_kd_;

  // IMU
  double imu_ang_vel_[3];
  double imu_lin_acc_[3];
  double imu_orientation_[4];
};

}  // namespace humanoid_hardware
