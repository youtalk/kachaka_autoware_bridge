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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "kachaka_autoware_vehicle_interface/operation_mode_state_machine.hpp"

using kachaka_autoware_vehicle_interface::OperationMode;
using kachaka_autoware_vehicle_interface::OperationModeStateMachine;

TEST(OperationModeStateMachine, InitialStateIsStop)
{
  OperationModeStateMachine sm;
  EXPECT_EQ(sm.get_state(), OperationMode::STOP);
}

TEST(OperationModeStateMachine, RequestAutonomousFromStopSucceeds)
{
  OperationModeStateMachine sm;
  EXPECT_TRUE(sm.request_autonomous());
  EXPECT_EQ(sm.get_state(), OperationMode::AUTONOMOUS);
}

TEST(OperationModeStateMachine, RequestStopFromAutonomousSucceeds)
{
  OperationModeStateMachine sm;
  sm.request_autonomous();
  EXPECT_TRUE(sm.request_stop());
  EXPECT_EQ(sm.get_state(), OperationMode::STOP);
}

TEST(OperationModeStateMachine, RequestSameStateIsIdempotent)
{
  OperationModeStateMachine sm;
  EXPECT_TRUE(sm.request_stop());
  EXPECT_EQ(sm.get_state(), OperationMode::STOP);
  sm.request_autonomous();
  EXPECT_TRUE(sm.request_autonomous());
  EXPECT_EQ(sm.get_state(), OperationMode::AUTONOMOUS);
}

TEST(OperationModeStateMachine, ConcurrentReadersAndWritersConverge)
{
  // Stress the lock: two writer threads alternately request AUTONOMOUS / STOP
  // while two reader threads continuously sample get_state(). After joining,
  // the final state must be one of the two valid enum values (i.e. no torn
  // read or undefined state). Without the internal mutex, TSan / ASan builds
  // would flag the data race on state_; this test additionally guards against
  // accidentally removing the lock in the future.
  OperationModeStateMachine sm;
  std::atomic<bool> stop{false};
  std::atomic<int> observed_invalid{0};

  std::vector<std::thread> threads;
  threads.emplace_back(
    [&] {
      while (!stop.load(std::memory_order_relaxed)) {
        sm.request_autonomous();
      }
    });
  threads.emplace_back(
    [&] {
      while (!stop.load(std::memory_order_relaxed)) {
        sm.request_stop();
      }
    });
  for (int i = 0; i < 2; ++i) {
    threads.emplace_back(
      [&] {
        while (!stop.load(std::memory_order_relaxed)) {
          const auto s = sm.get_state();
          if (s != OperationMode::STOP && s != OperationMode::AUTONOMOUS) {
            observed_invalid.fetch_add(1, std::memory_order_relaxed);
          }
        }
      });
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  stop.store(true, std::memory_order_relaxed);
  for (auto & t : threads) {
    t.join();
  }

  EXPECT_EQ(observed_invalid.load(), 0);
  const auto final_state = sm.get_state();
  EXPECT_TRUE(
    final_state == OperationMode::STOP || final_state == OperationMode::AUTONOMOUS);
}
