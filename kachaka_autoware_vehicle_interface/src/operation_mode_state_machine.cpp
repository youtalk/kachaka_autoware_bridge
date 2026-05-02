// Copyright 2026 Yutaka Kondo
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

#include "kachaka_autoware_vehicle_interface/operation_mode_state_machine.hpp"

namespace kachaka_autoware_vehicle_interface {

OperationModeStateMachine::OperationModeStateMachine()
    : state_(OperationMode::STOP) {}

OperationMode OperationModeStateMachine::get_state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

bool OperationModeStateMachine::request_autonomous() {
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = OperationMode::AUTONOMOUS;
  return true;
}

bool OperationModeStateMachine::request_stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = OperationMode::STOP;
  return true;
}

}  // namespace kachaka_autoware_vehicle_interface
