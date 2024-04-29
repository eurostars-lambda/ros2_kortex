#pragma once

#include <rclcpp/macros.hpp>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <ActuatorConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <RouterClient.h>
#include <SessionManager.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

namespace k_api = Kinova::Api;

namespace kortex_driver {

enum class ControlMode { NONE = 0, POSITION, EFFORT };

enum class EffortControlMode { NONE = 0, TORQUE, TORQUE_HIGH_VELOCITY, CURRENT };

class KortexInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KortexInterface);

  virtual ~KortexInterface();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) final;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) final;

  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) final;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) final;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) final;

  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) final;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) final;

  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) final;

private:
  void increment_id();

  void send_joint_commands();

  k_api::TransportClientTcp* transport_tcp_;
  k_api::RouterClient* router_tcp_;
  k_api::SessionManager* session_manager_;
  k_api::TransportClientUdp* transport_udp_realtime_;
  k_api::RouterClient* router_udp_realtime_;
  k_api::SessionManager* session_manager_real_time_;

  k_api::Base::BaseClient* base_;
  k_api::BaseCyclic::BaseCyclicClient* base_cyclic_;
  k_api::ActuatorConfig::ActuatorConfigClient* actuator_config_;
  k_api::Base::ServoingModeInformation servoing_mode_;

  k_api::BaseCyclic::Command* base_command_;
  k_api::BaseCyclic::Feedback feedback_;
  std::size_t actuator_count_;
  std::vector<double> arm_commands_positions_;
  std::vector<double> arm_commands_efforts_;
  std::vector<double> arm_positions_;
  std::vector<double> arm_velocities_;
  std::vector<double> arm_efforts_;

  std::atomic<bool> block_write_;

  ControlMode stop_mode_;
  ControlMode start_mode_;
  ControlMode mode_;
  EffortControlMode effort_mode_;

  // fault control
  double in_fault_;

  std::vector<double> motor_constants_;
};

}// namespace kortex_driver
