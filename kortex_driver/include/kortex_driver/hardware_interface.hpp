// Copyright 2021, PickNik Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//----------------------------------------------------------------------
/*!\file
 *
 * \author Marq Rasmussen marq.rasmussen@picknik.ai
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2021-06-15
 *
 */
//----------------------------------------------------------------------
#ifndef KORTEX_DRIVER__HARDWARE_INTERFACE_HPP_
#define KORTEX_DRIVER__HARDWARE_INTERFACE_HPP_

#pragma once

#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "kortex_driver/visibility_control.h"

#include "ActuatorConfigClientRpc.h"
#include "BaseClientRpc.h"
#include "BaseCyclicClientRpc.h"
#include "RouterClient.h"
#include "SessionManager.h"
#include "TransportClientTcp.h"
#include "TransportClientUdp.h"

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace k_api = Kinova::Api;

namespace kortex_driver {

enum class ControlMode { NONE = 0, POSITION, EFFORT };

enum class EffortControlMode { NONE = 0, TORQUE, TORQUE_HIGH_VELOCITY, CURRENT };

class KortexMultiInterfaceHardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KortexMultiInterfaceHardware);

  virtual ~KortexMultiInterfaceHardware();

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) final;

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) final;

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) final;

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) final;

  KORTEX_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

  KORTEX_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

  KORTEX_DRIVER_PUBLIC
  return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) final;

  KORTEX_DRIVER_PUBLIC
  return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) final;

  KORTEX_DRIVER_PUBLIC
  return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) final;

  KORTEX_DRIVER_PUBLIC
  return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) final;

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
  k_api::BaseCyclic::Feedback feedback_;
  k_api::BaseCyclic::Command* base_command_;
  k_api::ActuatorConfig::ActuatorConfigClient* actuator_config_;
  std::size_t actuator_count_;

  k_api::Base::ServoingModeInformation servoing_mode_hw_;
  ControlMode stop_mode_;
  ControlMode start_mode_;
  ControlMode mode_;
  EffortControlMode effort_mode_;

  std::vector<double> arm_commands_positions_;
  std::vector<double> arm_commands_efforts_;
  std::vector<double> arm_positions_;
  std::vector<double> arm_velocities_;
  std::vector<double> arm_efforts_;
  std::vector<double> offset_;
  std::vector<double> motor_constants_;

  std::atomic<bool> block_write_;
  bool first_pass_;
  double in_fault_;
};

}// namespace kortex_driver

#endif// KORTEX_DRIVER__HARDWARE_INTERFACE_HPP_
