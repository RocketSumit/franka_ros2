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

#pragma once

#include <array>
#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <franka/model.h>
#include <franka/robot.h>
#include <franka_hardware/model.hpp>
#include <rclcpp/logger.hpp>

namespace franka_hardware {

enum class ControlMode {
  None = 0,
  JointTorque = (1 << 0),
  JointPosition = (1 << 1),
  // TODO: Add support for the following control modes.
  /* JointVelocity = (1 << 2), */
  /* CartesianVelocity = (1 << 3), */
  /* CartesianPose = (1 << 4), */
};

class Robot {
 public:
  /**
   * Connects to the robot. This method can block for up to one minute if the robot is not
   * responding. An exception will be thrown if the connection cannot be established.
   *
   * @param[in] robot_ip IP address or hostname of the robot.
   * @param[im] logger ROS Logger to print eventual warnings.
   */
  explicit Robot(const std::string& robot_ip, const rclcpp::Logger& logger);
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot& other) = delete;
  Robot& operator=(Robot&& other) = delete;
  Robot(Robot&& other) = delete;

  /// Stops the currently running loop and closes the connection with the robot.
  virtual ~Robot();

  /**
   * Starts a torque control loop. Before using this method make sure that no other
   * control or reading loop is currently active.
   */
  virtual void initializeTorqueControl();

  /**
   * Starts a position control loop. Before using this method make sure that no other
   * control or reading loop is currently active.
   */
  void initializePositionControl();

  /**
   * Starts a reading loop of the robot state. Before using this method make sure that no other
   * control or reading loop is currently active.
   */
  virtual void initializeContinuousReading();

  /// stops the control or reading loop of the robot.
  virtual void stopRobot();

  /**
   * Get the current robot state in a thread-safe way.
   * @return current robot state.
   */
  virtual franka::RobotState read();

  /**
   * Return pointer to the franka robot model object .
   * @return pointer to the current robot model.
   */
  virtual franka_hardware::Model* getModel();

  /**
   * Sends new desired torque commands to the control loop in a thread-safe way.
   * The robot will use these torques until a different set of torques are commanded.
   * @param[in] efforts torque command for each joint.
   */
  virtual void write_efforts(const std::array<double, 7>& efforts);

  /**
   * Sends new desired position commands to the control loop in a thread-safe way.
   * The robot will use these positions until a different set of positions are commanded.
   * @param[in] positions position command for each joint.
   */
  virtual void write_positions(const std::array<double, 7>& positions);

  /// @return true if there is no control or reading loop running.
  [[nodiscard]] virtual bool isStopped() const;

  /**
   * Wrapper around franka::Robot::automaticErrorRecovery(). This method is thread-safe.
   * Stops all currently running motions.
   *
   * If a control or motion generator loop is running in another thread, it will be preempted
   * with a franka::ControlException.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void resetError();

  /// @brief Check if the robot is connected.
  /// @return true if the robot is connected.
  bool connected();

  bool hasError() const { return has_error_; }

 protected:
  Robot() = default;

 private:
  std::unique_ptr<std::thread> control_thread_;
  std::unique_ptr<franka::Robot> robot_;
  std::unique_ptr<franka::Model> model_;
  std::unique_ptr<Model> franka_hardware_model_;

  std::mutex read_mutex_;
  std::mutex write_mutex_;
  std::atomic_bool finish_{false};
  bool stopped_ = true;
  franka::RobotState current_state_;

  std::array<double, 7> tau_command_{};
  std::array<double, 7> position_command_{};

  std::atomic_bool has_error_{false};
};
}  // namespace franka_hardware
