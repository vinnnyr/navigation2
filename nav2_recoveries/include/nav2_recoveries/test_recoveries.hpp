// Copyright (c) 2019 Intel Corporation
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
// limitations under the License. Reserved.
#include <chrono>
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_recoveries/recovery.hpp"
#include "nav2_msgs/action/dummy_recovery.hpp"

using nav2_recoveries::Recovery;
using nav2_recoveries::Status;
using RecoveryAction = nav2_msgs::action::DummyRecovery;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<RecoveryAction>;
using namespace std::chrono_literals;

// A recovery for testing the base class

class DummyRecovery : public Recovery<RecoveryAction>
{
public:
  DummyRecovery()
  : Recovery<RecoveryAction>(),
    initialized_(false) {}

  ~DummyRecovery() {}

  Status onRun(const std::shared_ptr<const RecoveryAction::Goal> goal) override
  {
    // A normal recovery would catch the command and initialize
    initialized_ = false;
    command_ = goal->command.data;
    start_time_ = std::chrono::system_clock::now();

    // onRun method can have various possible outcomes (success, failure, cancelled)
    // The output is defined by the tester class on the command string.
    if (command_ == "Testing success" || command_ == "Testing failure on run") {
      initialized_ = true;
      return Status::SUCCEEDED;
    }

    return Status::FAILED;
  }

  Status onCycleUpdate() override
  {
    // A normal recovery would set the robot in motion in the first call
    // and check for robot states on subsequent calls to check if the movement
    // was completed.

    if (command_ != "Testing success" || !initialized_) {
      return Status::FAILED;
    }

    // For testing, pretend the robot takes some fixed
    // amount of time to complete the motion.
    auto current_time = std::chrono::system_clock::now();
    auto motion_duration = 1s;

    if (current_time - start_time_ >= motion_duration) {
      // Movement was completed
      return Status::SUCCEEDED;
    }

    return Status::RUNNING;
  }

private:
  bool initialized_;
  std::string command_;
  std::chrono::system_clock::time_point start_time_;
};
