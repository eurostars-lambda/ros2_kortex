// #include <chrono>
// #include <cmath>
// #include <exception>
// #include <limits>
// #include <memory>
// #include <string>
// #include <vector>

#include "kortex_driver/kortex_interface.hpp"
#include "kortex_driver/kortex_math_util.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
const rclcpp::Logger LOGGER = rclcpp::get_logger("KortexInterface");
}

namespace kortex_driver {
// KortexInterface::KortexInterface()
// : servoing_mode_hw_(k_api::Base::ServoingModeInformation()),
//   first_pass_(true)
// {
//   // RCLCPP_INFO(LOGGER, "Setting severity threshold to DEBUG");
//   // auto ret = rcutils_logging_set_logger_level(LOGGER.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
//   // if (ret != RCUTILS_RET_OK)
//   // {
//   //   RCLCPP_ERROR(LOGGER, "Error setting severity: %s", rcutils_get_error_string().str);
//   //   rcutils_reset_error();
//   // }
// }

KortexInterface::~KortexInterface() {
  // If the controller manager is shutdown via Ctrl + C the on_deactivate methods won't be called.
  // We therefore need to make sure to actually deactivate the communication
  on_cleanup(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn KortexInterface::on_init(const hardware_interface::HardwareInfo& info) {
  RCLCPP_INFO(LOGGER, "Configuring Kortex Interface");

  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  info_ = info;
  actuator_count_ = 7;
  effort_mode_ = EffortControlMode::NONE;

  arm_positions_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_velocities_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_efforts_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_positions_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_efforts_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  motor_constants_ = std::vector<double>{11, 11, 11, 11, 7.6, 7.6, 7.6};

  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    if (joint.command_interfaces.size() != 2) {
      RCLCPP_FATAL(
          LOGGER, "Joint '%s' has %zu command interfaces. 2 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          LOGGER, "Joint '%s' has a %s command interface found as first command interface. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(
          LOGGER, "Joint '%s' has a %s command interface found as second command interface. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(
          LOGGER, "Joint '%s' has %zu state interface. 3 expected.", joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          LOGGER, "Joint '%s' has a %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          LOGGER, "Joint '%s' has a %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(
          LOGGER, "Joint '%s' has a %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(LOGGER, "Kortex Interface successfully configured");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KortexInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &arm_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &arm_velocities_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &arm_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KortexInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &arm_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &arm_commands_efforts_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn KortexInterface::on_configure(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(LOGGER, "Configuring Kortex Interface...");

  auto robot_ip = info_.hardware_parameters["robot_ip"];
  if (robot_ip.empty()) {
    RCLCPP_FATAL(LOGGER, "Robot ip is empty!");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(LOGGER, "Robot ip is '%s'", robot_ip.c_str());
  }
  auto username = info_.hardware_parameters["username"];
  if (username.empty()) {
    RCLCPP_FATAL(LOGGER, "Username is empty!");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(LOGGER, "Username is '%s'", username.c_str());
  }
  auto password = info_.hardware_parameters["password"];
  if (password.empty()) {
    RCLCPP_FATAL(LOGGER, "Password is empty!");
    return CallbackReturn::ERROR;
  }
  auto port = std::stoi(info_.hardware_parameters["port"]);
  if (port <= 0) {
    RCLCPP_FATAL(LOGGER, "Incorrect port number!");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(LOGGER, "Port used is '%d'", port);
  }
  auto port_realtime = std::stoi(info_.hardware_parameters["port_realtime"]);
  if (port_realtime <= 0) {
    RCLCPP_FATAL(LOGGER, "Incorrect realtime port number!");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(LOGGER, "Realtime port used is '%d'", port_realtime);
  }
  auto session_inactivity_timeout = std::stoi(info_.hardware_parameters["session_inactivity_timeout_ms"]);
  if (session_inactivity_timeout <= 0) {
    RCLCPP_FATAL(LOGGER, "Incorrect session inactivity timeout!");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(LOGGER, "Session inactivity timeout is '%d'", session_inactivity_timeout);
  }
  auto connection_inactivity_timeout = std::stoi(info_.hardware_parameters["connection_inactivity_timeout_ms"]);
  if (connection_inactivity_timeout <= 0) {
    RCLCPP_FATAL(LOGGER, "Incorrect connection inactivity timeout!");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(LOGGER, "Connection inactivity timeout is '%d'", connection_inactivity_timeout);
  }

  RCLCPP_INFO_STREAM(LOGGER, "Connecting to robot at " << robot_ip);

  auto error_callback = [](k_api::KError err) {
    cout << "_________ callback error _________" << err.toString();
  };
  transport_tcp_ = new k_api::TransportClientTcp();
  router_tcp_ = new k_api::RouterClient(transport_tcp_, error_callback);
  transport_tcp_->connect(robot_ip, port);

  transport_udp_realtime_ = new k_api::TransportClientUdp();
  router_udp_realtime_ = new k_api::RouterClient(transport_udp_realtime_, error_callback);
  transport_udp_realtime_->connect(robot_ip, port_realtime);

  // Set session data connection information
  auto create_session_info = k_api::Session::CreateSessionInfo();
  create_session_info.set_username(username);
  create_session_info.set_password(password);
  create_session_info.set_session_inactivity_timeout(session_inactivity_timeout);      // (milliseconds)
  create_session_info.set_connection_inactivity_timeout(connection_inactivity_timeout);// (milliseconds)

  // Session manager service wrapper
  RCLCPP_INFO(LOGGER, "Creating sessions for communication");
  session_manager_ = new k_api::SessionManager(router_tcp_);
  session_manager_->CreateSession(create_session_info);
  session_manager_real_time_ = new k_api::SessionManager(router_udp_realtime_);
  session_manager_real_time_->CreateSession(create_session_info);
  RCLCPP_INFO(LOGGER, "Sessions created");

  base_ = new k_api::Base::BaseClient(router_tcp_);
  base_cyclic_ = new k_api::BaseCyclic::BaseCyclicClient(router_udp_realtime_);
  actuator_config_ = new k_api::ActuatorConfig::ActuatorConfigClient(router_tcp_);

  // reset faults on activation, go back to low level servoing after
  servoing_mode_.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  base_->SetServoingMode(servoing_mode_);
  try {
    base_->ClearFaults();
  } catch (k_api::KDetailedException& ex) {
    RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());
    RCLCPP_ERROR_STREAM(
        LOGGER,
        "Error sub-code: " << k_api::SubErrorCodes_Name(
            k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
  }

  auto actuator_count = base_->GetActuatorCount().count();
  if (actuator_count != actuator_count_) {
    RCLCPP_FATAL(LOGGER, "Actuator count reported by robot is '%u', expected '%lu", actuator_count, actuator_count_);
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(LOGGER, "Kortex Interface successfully configured");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KortexInterface::on_cleanup(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(LOGGER, "Cleaning up Kortex Interface...");

  // Close API session
  session_manager_->CloseSession();
  session_manager_real_time_->CloseSession();

  // Deactivate the router and cleanly disconnect from the transport object
  router_tcp_->SetActivationStatus(false);
  transport_tcp_->disconnect();
  router_udp_realtime_->SetActivationStatus(false);
  transport_udp_realtime_->disconnect();

  delete base_;
  delete base_cyclic_;
  delete session_manager_;
  delete session_manager_real_time_;
  delete router_tcp_;
  delete router_udp_realtime_;
  delete transport_tcp_;
  delete transport_udp_realtime_;

  RCLCPP_INFO(LOGGER, "Kortex Interface successfully cleaned up");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KortexInterface::on_activate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(LOGGER, "Activating Kortex Interface...");
  // first read
  auto base_feedback = base_cyclic_->RefreshFeedback();
  base_command_ = new k_api::BaseCyclic::Command();

  // Add each actuator to the base_command_ and set the command to its current position
  for (std::size_t i = 0; i < actuator_count_; i++) {
    base_command_->add_actuators()->set_position(base_feedback.actuators(i).position());
  }

  // Send a first frame
  base_feedback = base_cyclic_->Refresh(*base_command_);

  RCLCPP_INFO(LOGGER, "Kortex Interface successfully activated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KortexInterface::on_deactivate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(LOGGER, "Deactivating Kortex Interface...");

  servoing_mode_.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  base_->SetServoingMode(servoing_mode_);

  RCLCPP_INFO(LOGGER, "Kortex Interface successfully deactivated!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type KortexInterface::read(const rclcpp::Time&, const rclcpp::Duration&) {
  in_fault_ = (feedback_.base().active_state() == k_api::Common::ArmState::ARMSTATE_IN_FAULT);

  for (std::size_t i = 0; i < actuator_count_; i++) {
    auto num_turns_tmp = 0;
    arm_positions_[i] = KortexMathUtil::wrapRadiansFromMinusPiToPi(
        KortexMathUtil::toRad(feedback_.actuators(i).position()), num_turns_tmp); // rad
    arm_velocities_[i] = KortexMathUtil::toRad(feedback_.actuators(i).velocity());// rad/sec
    arm_efforts_[i] = feedback_.actuators(i).torque();                            // N*m

    in_fault_ += (feedback_.actuators(i).fault_bank_a() + feedback_.actuators(i).fault_bank_b());
  }

  // add all base's faults and warnings into series
  in_fault_ += (feedback_.base().fault_bank_a() + feedback_.base().fault_bank_b());

  // add mode that can't be easily reached
  in_fault_ += (feedback_.base().active_state() == k_api::Common::ARMSTATE_SERVOING_READY);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KortexInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
  if (block_write_) {
    feedback_ = base_cyclic_->RefreshFeedback();
    return hardware_interface::return_type::OK;
  }

  // if (!std::isnan(reset_fault_cmd_) && fault_controller_running_)
  // {
  //   try
  //   {
  //     // change servoing mode first
  //     servoing_mode_hw_.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  //     base_->SetServoingMode(servoing_mode_hw_);
  //     // apply emergency stop - twice to make it sure as calling it once appeared to be unreliable
  //     // (detected by testing)
  //     base_->ApplyEmergencyStop(0, {false, 0, 100});
  //     base_->ApplyEmergencyStop(0, {false, 0, 100});
  //     // clear faults
  //     base_->ClearFaults();
  //     // back to original servoing mode
  //     if (
  //       arm_mode_ == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING ||
  //       arm_mode_ == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING)
  //     {
  //       servoing_mode_hw_.set_servoing_mode(arm_mode_);
  //       base_->SetServoingMode(servoing_mode_hw_);
  //     }
  //     reset_fault_async_success_ = 1.0;
  //   }
  //   catch (k_api::KDetailedException & ex)
  //   {
  //     RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

  //     RCLCPP_ERROR_STREAM(
  //       LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
  //                 k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
  //     reset_fault_async_success_ = 0.0;
  //   }
  //   catch (...)
  //   {
  //     reset_fault_async_success_ = 0.0;
  //   }
  //   reset_fault_cmd_ = NO_CMD;
  // }

  if (in_fault_ == 0.0 && (feedback_.base().active_state() == k_api::Common::ARMSTATE_SERVOING_LOW_LEVEL)) {
    increment_id();
    send_joint_commands();
  } else {
    // this is needed when the robot was faulted so we can internally conclude it is not faulted anymore
    feedback_ = base_cyclic_->RefreshFeedback();
  }

  return hardware_interface::return_type::OK;
}

void KortexInterface::send_joint_commands() {
  switch (mode_) {
    case ControlMode::POSITION: {
      for (std::size_t i = 0; i < actuator_count_; ++i) {
        auto cmd_degrees_tmp = static_cast<float>(
            KortexMathUtil::wrapDegreesFromZeroTo360(KortexMathUtil::toDeg(arm_commands_positions_[i])));
        base_command_->mutable_actuators(static_cast<int>(i))->set_position(cmd_degrees_tmp);
        base_command_->mutable_actuators(static_cast<int>(i))->set_command_id(base_command_->frame_id());
      }
      break;
    }
    case ControlMode::EFFORT: {
      for (std::size_t i = 0; i < actuator_count_; ++i) {
        base_command_->mutable_actuators(i)->set_position(feedback_.actuators(i).position());
        base_command_->mutable_actuators(static_cast<int>(i))->set_command_id(base_command_->frame_id());
        // TODO: check limits
        switch (effort_mode_) {
          case EffortControlMode::TORQUE:
          case EffortControlMode::TORQUE_HIGH_VELOCITY:
            base_command_->mutable_actuators(i)->set_torque_joint(arm_commands_efforts_[i]);
            break;
          case EffortControlMode::CURRENT:
            base_command_->mutable_actuators(i)->set_current_motor(arm_commands_efforts_[i] / motor_constants_[i]);
            break;
          default:
            break;
        }
      }
      break;
    }
    default:
      break;
  }

  try {
    feedback_ = base_cyclic_->Refresh(*base_command_);
  } catch (k_api::KDetailedException& ex) {
    feedback_ = base_cyclic_->RefreshFeedback();
    RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

    RCLCPP_ERROR_STREAM(
        LOGGER,
        "Error sub-code: " << k_api::SubErrorCodes_Name(
            k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
  } catch (std::runtime_error& ex_runtime) {
    feedback_ = base_cyclic_->RefreshFeedback();
    RCLCPP_ERROR_STREAM(LOGGER, "Runtime error: " << ex_runtime.what());
  } catch (std::future_error& ex_future) {
    feedback_ = base_cyclic_->RefreshFeedback();
    RCLCPP_ERROR_STREAM(LOGGER, "Future error: " << ex_future.what());
  } catch (std::exception& ex_std) {
    feedback_ = base_cyclic_->RefreshFeedback();
    RCLCPP_ERROR_STREAM(LOGGER, "Standard exception: " << ex_std.what());
  }
}

void KortexInterface::increment_id() {
  // Incrementing identifier ensures actuators can reject out of time frames
  base_command_->set_frame_id(base_command_->frame_id() + 1);
  if (base_command_->frame_id() > 65535)
    base_command_->set_frame_id(0);
}

hardware_interface::return_type KortexInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) {
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  // sleep to ensure all outgoing write commands have finished
  block_write_ = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  start_mode_ = ControlMode::NONE;
  stop_mode_ = ControlMode::NONE;
  auto start_modes = std::vector<ControlMode>();
  auto stop_modes = std::vector<ControlMode>();

  // Starting interfaces
  // add start interface per joint in tmp var for later check
  for (const auto& key : start_interfaces) {
    for (auto i = 0u; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        start_modes.push_back(ControlMode::POSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
        start_modes.push_back(ControlMode::EFFORT);
      }
    }
  }
  if (start_modes.size() != 0) {
    // set new mode to all interfaces at the same time
    if (start_modes.size() != actuator_count_) {
      ret_val = hardware_interface::return_type::ERROR;
    }
    // all start interfaces must be the same
    if (!std::equal(start_modes.begin() + 1, start_modes.end(), start_modes.begin())) {
      ret_val = hardware_interface::return_type::ERROR;
    }
    start_mode_ = start_modes.at(0);
  }

  // Stopping interfaces
  // add stop interface per joint in tmp var for later check
  for (const auto& key : stop_interfaces) {
    for (auto i = 0u; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        stop_modes.push_back(ControlMode::POSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
        stop_modes.push_back(ControlMode::EFFORT);
      }
    }
  }
  if (stop_modes.size() != 0) {
    if (stop_modes.size() != 6) {
      ret_val = hardware_interface::return_type::ERROR;
    }
    // all start interfaces must be the same
    if (!std::equal(stop_modes.begin() + 1, stop_modes.end(), stop_modes.begin())) {
      ret_val = hardware_interface::return_type::ERROR;
    }
    stop_mode_ = stop_modes.at(0);
  }
  return ret_val;
}

hardware_interface::return_type
KortexInterface::perform_command_mode_switch(const vector<std::string>&, const vector<std::string>&) {
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  switch (stop_mode_) {
    case ControlMode::POSITION:
      mode_ = ControlMode::NONE;
      arm_commands_positions_ = arm_positions_;
      break;
    case ControlMode::EFFORT:
      mode_ = ControlMode::NONE;
      arm_commands_efforts_ = std::vector<double>(actuator_count_, std::numeric_limits<double>::quiet_NaN());
      break;
    default:
      break;
  }

  switch (start_mode_) {
    case ControlMode::POSITION: {
      servoing_mode_.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
      base_->SetServoingMode(servoing_mode_);
      arm_commands_positions_ = arm_positions_;
      arm_commands_efforts_ = std::vector<double>(actuator_count_, std::numeric_limits<double>::quiet_NaN());
      auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
      control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
      for (std::size_t id = 1; id < actuator_count_ + 1; ++id) {
        actuator_config_->SetControlMode(control_mode_message, id);
      }
      feedback_ = base_cyclic_->RefreshFeedback();
      mode_ = ControlMode::POSITION;
      break;
    }
    case ControlMode::EFFORT: {
      servoing_mode_.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
      base_->SetServoingMode(servoing_mode_);
      arm_commands_positions_ = arm_positions_;
      arm_commands_efforts_ = std::vector<double>(actuator_count_, std::numeric_limits<double>::quiet_NaN());
      auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
      switch (effort_mode_) {
        case EffortControlMode::TORQUE:
          control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
          break;
        case EffortControlMode::TORQUE_HIGH_VELOCITY:
          control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE_HIGH_VELOCITY);
          break;
        case EffortControlMode::CURRENT:
          control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::CURRENT);
          break;
        default:
          break;
      }
      for (std::size_t id = 1; id < actuator_count_ + 1; ++id) {
        actuator_config_->SetControlMode(control_mode_message, id);
      }
      feedback_ = base_cyclic_->RefreshFeedback();
      mode_ = ControlMode::EFFORT;
      break;
    }
    default:
      mode_ = ControlMode::NONE;
      break;
  }

  start_mode_ = ControlMode::NONE;
  stop_mode_ = ControlMode::NONE;
  block_write_ = false;

  return ret_val;
}

}// namespace kortex_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(kortex_driver::KortexInterface, hardware_interface::SystemInterface)
