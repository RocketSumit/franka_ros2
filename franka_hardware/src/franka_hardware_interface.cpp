// Copyright (c) 2021 Franka Emika GmbH
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

#include <franka_hardware/franka_hardware_interface.hpp>

#include <algorithm>
#include <cmath>
#include <exception>

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>
#include <franka/exception.h>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/scope_exit.hpp>
#include <rcpputils/scope_exit.hpp>

namespace franka_hardware {

using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

FrankaHardwareInterface::FrankaHardwareInterface(std::unique_ptr<Robot> robot)
    : robot_{std::move(robot)} {}

std::vector<StateInterface> FrankaHardwareInterface::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;
  for (auto i = 0U; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_.at(i)));
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_.at(i)));
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_.at(i)));
  }

  state_interfaces.emplace_back(StateInterface(
      k_robot_name, k_robot_state_interface_name,
      reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
          &hw_franka_robot_state_addr_)));
  state_interfaces.emplace_back(StateInterface(
      k_robot_name, k_robot_model_interface_name,
      reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
          &hw_franka_model_ptr_)));

  for (auto& sensor : info_.sensors) {
    for (uint j = 0; j < sensor.state_interfaces.size(); ++j) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          sensor.name, sensor.state_interfaces[j].name, &hw_ft_sensor_measurements_[j]));
    }
  }

  // state interface which reports if robot is faulted
  state_interfaces.emplace_back(
      hardware_interface::StateInterface("reset_fault", "internal_fault", &in_fault_));

  return state_interfaces;
}

std::vector<CommandInterface> FrankaHardwareInterface::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  for (auto i = 0U; i < info_.joints.size(); i++) {
    for (const auto& command_interface : info_.joints[i].command_interfaces) {
      if (command_interface.name == hardware_interface::HW_IF_POSITION) {
        command_interfaces.emplace_back(CommandInterface(
            info_.joints[i].name, command_interface.name, &position_commands_.at(i)));
      }
      if (command_interface.name == hardware_interface::HW_IF_EFFORT) {
        command_interfaces.emplace_back(CommandInterface(
            info_.joints[i].name, command_interface.name, &effort_commands_.at(i)));
      }
    }
  }

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface("reset_fault", "command", &reset_fault_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "reset_fault", "async_success", &reset_fault_async_success_));

  return command_interfaces;
}

CallbackReturn FrankaHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_->initializeContinuousReading();
  effort_commands_.fill(0);
  position_commands_ = hw_positions_;
  read(rclcpp::Time(0),
       rclcpp::Duration(0, 0));  // makes sure that the robot state is properly initialized.
  RCLCPP_INFO(getLogger(), "Started");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(getLogger(), "trying to Stop...");
  robot_->stopRobot();
  RCLCPP_INFO(getLogger(), "Stopped");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type FrankaHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) {
  if (robot_->connected()) {
    if (hw_franka_model_ptr_ == nullptr) {
      hw_franka_model_ptr_ = robot_->getModel();
    }
    hw_franka_robot_state_ = robot_->read();
    hw_positions_ = hw_franka_robot_state_.q;
    hw_velocities_ = hw_franka_robot_state_.dq;
    hw_efforts_ = hw_franka_robot_state_.tau_J;
    hw_ft_sensor_measurements_ = hw_franka_robot_state_.K_F_ext_hat_K;
    if (control_mode_ == franka_hardware::ControlMode::None) {
      position_commands_ = hw_positions_;
    }
    if (robot_->hasError()) {
      in_fault_ = 1.0;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/) {
  if (std::any_of(position_commands_.begin(), position_commands_.end(),
                  [](double c) { return not std::isfinite(c); })) {
    return hardware_interface::return_type::ERROR;
  }
  if (std::any_of(effort_commands_.begin(), effort_commands_.end(),
                  [](double c) { return not std::isfinite(c); })) {
    return hardware_interface::return_type::ERROR;
  }

  if (!std::isnan(reset_fault_cmd_) && fault_controller_running_) {
    try {
      if (robot_->hasError()) {
        robot_->resetError();
        RCLCPP_INFO(getLogger(), "Recovered from error");
      }
      reset_fault_async_success_ = 1.0;
      in_fault_ = 0.0;
    } catch (const franka::Exception& ex) {
      reset_fault_async_success_ = 0.0;
    }
    reset_fault_cmd_ = std::numeric_limits<double>::quiet_NaN();
  }

  if (control_mode_ == franka_hardware::ControlMode::JointTorque) {
    robot_->write_efforts(effort_commands_);
  } else if (control_mode_ == franka_hardware::ControlMode::JointPosition) {
    robot_->write_positions(position_commands_);
  }
  return hardware_interface::return_type::OK;
}

CallbackReturn FrankaHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (info_.joints.size() != kNumberOfJoints) {
    RCLCPP_FATAL(getLogger(), "Got %ld joints. Expected %ld.", info_.joints.size(),
                 kNumberOfJoints);
    return CallbackReturn::ERROR;
  }

  for (const auto& joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 && joint.command_interfaces.size() != 2) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu command interfaces found. 1 or 2 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }
    for (const auto& command_interface : joint.command_interfaces) {
      if (command_interface.name != hardware_interface::HW_IF_EFFORT and
          command_interface.name != hardware_interface::HW_IF_POSITION) {
        RCLCPP_FATAL(getLogger(),
                     "Joint '%s' has unexpected command interface '%s'. Expected '%s' or '%s'",
                     joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                     hardware_interface::HW_IF_EFFORT, hardware_interface::HW_IF_POSITION);
        return CallbackReturn::ERROR;
      }
    }
    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu state interfaces found. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
    }
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.state_interfaces[2].name.c_str(),
                   hardware_interface::HW_IF_EFFORT);
    }
  }
  if (!robot_) {
    std::string robot_ip;
    try {
      robot_ip = info_.hardware_parameters.at("robot_ip");
    } catch (const std::out_of_range& ex) {
      RCLCPP_FATAL(getLogger(), "Parameter 'robot_ip' is not set");
      return CallbackReturn::ERROR;
    }
    try {
      RCLCPP_INFO(getLogger(), "Connecting to robot at \"%s\" ...", robot_ip.c_str());
      robot_ = std::make_shared<Robot>(robot_ip, getLogger());
    } catch (const franka::Exception& e) {
      RCLCPP_FATAL(getLogger(), "Could not connect to robot");
      RCLCPP_FATAL(getLogger(), "%s", e.what());
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(getLogger(), "Successfully connected to robot");
  }

  node_ = std::make_shared<FrankaParamServiceServer>(rclcpp::NodeOptions(), robot_);
  executor_ = std::make_shared<FrankaExecutor>();
  executor_->add_node(node_);
  return CallbackReturn::SUCCESS;
}

rclcpp::Logger FrankaHardwareInterface::getLogger() {
  return rclcpp::get_logger("FrankaHardwareInterface");
}

hardware_interface::return_type FrankaHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/) {
  RCLCPP_SCOPE_EXIT({
    start_modes_.clear();
    stop_modes_.clear();
  });

  if (stop_modes_.size() != 0 &&
      std::find(stop_modes_.begin(), stop_modes_.end(), hardware_interface::HW_IF_POSITION) !=
          stop_modes_.end()) {
    position_interface_claimed_ = false;
  } else if (stop_modes_.size() != 0 &&
             std::find(stop_modes_.begin(), stop_modes_.end(), hardware_interface::HW_IF_EFFORT) !=
                 stop_modes_.end()) {
    effort_interface_claimed_ = false;
  }

  if (start_modes_.size() != 0 &&
      std::find(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_POSITION) !=
          start_modes_.end()) {
    position_interface_claimed_ = true;
  } else if (start_modes_.size() != 0 &&
             std::find(start_modes_.begin(), start_modes_.end(),
                       hardware_interface::HW_IF_EFFORT) != start_modes_.end()) {
    effort_interface_claimed_ = true;
  }

  if (effort_interface_claimed_) {
    robot_->stopRobot();
    robot_->initializeTorqueControl();
    control_mode_ = franka_hardware::ControlMode::JointTorque;
  } else if (position_interface_claimed_) {
    robot_->stopRobot();
    robot_->initializePositionControl();
    control_mode_ = franka_hardware::ControlMode::JointPosition;
  } else if (control_mode_ != franka_hardware::ControlMode::None and
             not effort_interface_claimed_ and not position_interface_claimed_) {
    robot_->stopRobot();
    robot_->initializeContinuousReading();
    control_mode_ = franka_hardware::ControlMode::None;
  }

  if (stop_fault_controller_) {
    fault_controller_running_ = false;
  }
  if (start_fault_controller_) {
    fault_controller_running_ = true;
  }

  stop_fault_controller_ = false;
  start_fault_controller_ = false;
  start_modes_.clear();
  stop_modes_.clear();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  stop_modes_.clear();
  start_modes_.clear();
  // Starting interfaces
  // add start interface per joint in tmp var for later check
  for (const auto& key : start_interfaces) {
    for (auto i = 0u; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        start_modes_.push_back(hardware_interface::HW_IF_POSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
        start_modes_.push_back(hardware_interface::HW_IF_EFFORT);
      }
    }
  }
  // set new mode to all interfaces at the same time
  if (!start_modes_.empty() && start_modes_.size() != kNumberOfJoints) {
    return hardware_interface::return_type::ERROR;
  }

  // reset auxiliary switching booleans
  stop_fault_controller_ = false;
  start_fault_controller_ = false;

  // all start interfaces must be the same - can't mix position and velocity control
  if (!start_modes_.empty()) {
    if (!std::equal(start_modes_.begin() + 1, start_modes_.end(), start_modes_.begin())) {
      return hardware_interface::return_type::ERROR;
    }
    if (robot_->hasError()) {
      RCLCPP_ERROR(
          getLogger(),
          "Robot is in error state. Cannot switch control mode -- Make sure to call ros2 service "
          "call /NAMESPACE/fault_controller/reset_fault example_interfaces/srv/Trigger.");
      return hardware_interface::return_type::ERROR;
    }

    if (start_modes_[0] == hardware_interface::HW_IF_POSITION && effort_interface_claimed_) {
      RCLCPP_ERROR(getLogger(),
                   "Cannot switch to position control mode. Effort interface is claimed.");
      return hardware_interface::return_type::ERROR;
    }
    if (start_modes_[0] == hardware_interface::HW_IF_EFFORT && position_interface_claimed_) {
      RCLCPP_ERROR(getLogger(),
                   "Cannot switch to effort control mode. Position interface is claimed.");
      return hardware_interface::return_type::ERROR;
    }
  }

  // Stopping interfaces
  // add stop interface per joint in tmp var for later check
  for (const auto& key : stop_interfaces) {
    for (auto i = 0u; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        stop_modes_.push_back(hardware_interface::HW_IF_POSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
        stop_modes_.push_back(hardware_interface::HW_IF_EFFORT);
      }
    }
  }
  // stop all interfaces at the same time
  if (stop_modes_.size() != 0 &&
      (stop_modes_.size() != kNumberOfJoints ||
       !std::equal(stop_modes_.begin() + 1, stop_modes_.end(), stop_modes_.begin()))) {
    return hardware_interface::return_type::ERROR;
  }

  for (const auto& key : stop_interfaces) {
    if ((key == "reset_fault/command") || (key == "reset_fault/async_success")) {
      stop_fault_controller_ = true;
    }
  }
  for (const auto& key : start_interfaces) {
    if ((key == "reset_fault/command") || (key == "reset_fault/async_success")) {
      start_fault_controller_ = true;
    }
  }
  return hardware_interface::return_type::OK;
}
}  // namespace franka_hardware

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_hardware::FrankaHardwareInterface,
                       hardware_interface::SystemInterface)
