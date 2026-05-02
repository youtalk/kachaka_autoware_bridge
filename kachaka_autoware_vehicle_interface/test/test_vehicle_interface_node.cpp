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

#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <optional>
#include <thread>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "kachaka_autoware_vehicle_interface/vehicle_interface_node.hpp"

using namespace std::chrono_literals;
using kachaka_autoware_vehicle_interface::VehicleInterfaceNode;
using ChangeOperationMode = autoware_adapi_v1_msgs::srv::ChangeOperationMode;
using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;

namespace {

// Spin both the node-under-test and the helper node together until `predicate`
// becomes true or the deadline elapses. Returns true when the predicate fired.
template <typename Predicate>
bool spin_until(rclcpp::Executor& executor, std::chrono::milliseconds timeout,
                Predicate predicate) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  executor.spin_some();
  return predicate();
}

class VehicleInterfaceNodeFixture : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::NodeOptions options;
    options.append_parameter_override("auto_enable_manual_control", false);
    options.append_parameter_override("publish_period_velocity_status", 0.02);
    options.append_parameter_override("publish_period_operation_mode", 0.05);
    options.append_parameter_override("cmd_vel_timeout", 0.2);
    node_ = std::make_shared<VehicleInterfaceNode>(options);
    helper_ = std::make_shared<rclcpp::Node>("vehicle_interface_test_helper");

    control_pub_ =
        helper_->create_publisher<autoware_control_msgs::msg::Control>(
            "/control/command/control_cmd", rclcpp::QoS(1));
    odom_pub_ = helper_->create_publisher<nav_msgs::msg::Odometry>(
        "/kachaka/wheel_odometry/wheel_odometry", rclcpp::SensorDataQoS());
    twist_sub_ = helper_->create_subscription<geometry_msgs::msg::Twist>(
        "/kachaka/manual_control/cmd_vel", rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          last_twist_ = *msg;
        });
    velocity_status_sub_ = helper_->create_subscription<
        autoware_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_status", rclcpp::QoS(1),
        [this](
            const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg) {
          last_velocity_status_ = *msg;
        });
    op_mode_sub_ = helper_->create_subscription<OperationModeState>(
        "/system/operation_mode/state", rclcpp::QoS(1).transient_local(),
        [this](const OperationModeState::SharedPtr msg) {
          last_op_mode_ = *msg;
        });
    change_to_autonomous_client_ = helper_->create_client<ChangeOperationMode>(
        "/system/operation_mode/change_to_autonomous");
    change_to_stop_client_ = helper_->create_client<ChangeOperationMode>(
        "/system/operation_mode/change_to_stop");

    executor_.add_node(node_);
    executor_.add_node(helper_);
  }

  void TearDown() override {
    executor_.remove_node(helper_);
    executor_.remove_node(node_);
  }

  bool change_mode(rclcpp::Client<ChangeOperationMode>::SharedPtr client) {
    if (!client->wait_for_service(1s)) {
      return false;
    }
    auto req = std::make_shared<ChangeOperationMode::Request>();
    auto future = client->async_send_request(req);
    return spin_until(executor_, 1s, [&future] {
      return future.wait_for(0s) == std::future_status::ready;
    });
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<VehicleInterfaceNode> node_;
  std::shared_ptr<rclcpp::Node> helper_;

  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr
      control_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
      velocity_status_sub_;
  rclcpp::Subscription<OperationModeState>::SharedPtr op_mode_sub_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr change_to_autonomous_client_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr change_to_stop_client_;

  std::optional<geometry_msgs::msg::Twist> last_twist_;
  std::optional<autoware_vehicle_msgs::msg::VelocityReport>
      last_velocity_status_;
  std::optional<OperationModeState> last_op_mode_;
};

}  // namespace

TEST_F(VehicleInterfaceNodeFixture, OpModeStartsInStop) {
  ASSERT_TRUE(
      spin_until(executor_, 1s, [this] { return last_op_mode_.has_value(); }));
  EXPECT_EQ(last_op_mode_->mode, OperationModeState::STOP);
  EXPECT_FALSE(last_op_mode_->is_autoware_control_enabled);
}

TEST_F(VehicleInterfaceNodeFixture, ControlIsBlockedWhileStop) {
  // Without changing to AUTONOMOUS, publish a control_cmd and confirm no Twist
  // appears.
  ASSERT_TRUE(spin_until(executor_, 500ms,
                         [this] { return last_op_mode_.has_value(); }));

  autoware_control_msgs::msg::Control cmd;
  cmd.longitudinal.velocity = 0.2f;
  control_pub_->publish(cmd);

  // Spin briefly, then assert no Twist was received.
  spin_until(executor_, 200ms, [] { return false; });
  EXPECT_FALSE(last_twist_.has_value());
}

TEST_F(VehicleInterfaceNodeFixture, AutonomousGatePassesControlToTwist) {
  ASSERT_TRUE(change_mode(change_to_autonomous_client_));
  ASSERT_TRUE(spin_until(executor_, 500ms, [this] {
    return last_op_mode_ &&
           last_op_mode_->mode == OperationModeState::AUTONOMOUS;
  }));

  autoware_control_msgs::msg::Control cmd;
  cmd.longitudinal.velocity = 0.15f;
  cmd.lateral.steering_tire_angle = 0.0f;
  control_pub_->publish(cmd);

  ASSERT_TRUE(
      spin_until(executor_, 500ms, [this] { return last_twist_.has_value(); }));
  EXPECT_NEAR(last_twist_->linear.x, 0.15, 1e-5);
  EXPECT_NEAR(last_twist_->angular.z, 0.0, 1e-5);
}

TEST_F(VehicleInterfaceNodeFixture, ChangeToStopBlocksFurtherControl) {
  ASSERT_TRUE(change_mode(change_to_autonomous_client_));
  ASSERT_TRUE(spin_until(executor_, 500ms, [this] {
    return last_op_mode_ &&
           last_op_mode_->mode == OperationModeState::AUTONOMOUS;
  }));

  autoware_control_msgs::msg::Control cmd;
  cmd.longitudinal.velocity = 0.1f;
  control_pub_->publish(cmd);
  ASSERT_TRUE(
      spin_until(executor_, 500ms, [this] { return last_twist_.has_value(); }));

  ASSERT_TRUE(change_mode(change_to_stop_client_));
  ASSERT_TRUE(spin_until(executor_, 500ms, [this] {
    return last_op_mode_ && last_op_mode_->mode == OperationModeState::STOP;
  }));

  last_twist_.reset();
  cmd.longitudinal.velocity = 0.25f;
  control_pub_->publish(cmd);
  // Allow up to 2x the cmd_vel_timeout (0.2s) so the timer can fire if it
  // would, then assert no new Twist was received (timeout zero-Twist is also
  // gated by the AUTONOMOUS check, so STOP remains silent).
  spin_until(executor_, 400ms, [] { return false; });
  EXPECT_FALSE(last_twist_.has_value());
}

TEST_F(VehicleInterfaceNodeFixture, OdometryRepublishedAsVelocityStatus) {
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "odom";
  odom.twist.twist.linear.x = 0.123;
  odom.twist.twist.angular.z = 0.456;
  odom_pub_->publish(odom);

  ASSERT_TRUE(spin_until(executor_, 500ms,
                         [this] { return last_velocity_status_.has_value(); }));
  EXPECT_FLOAT_EQ(last_velocity_status_->longitudinal_velocity, 0.123f);
  EXPECT_FLOAT_EQ(last_velocity_status_->lateral_velocity, 0.0f);
  EXPECT_FLOAT_EQ(last_velocity_status_->heading_rate, 0.456f);
}

TEST_F(VehicleInterfaceNodeFixture, CmdVelTimeoutPublishesZeroTwist) {
  ASSERT_TRUE(change_mode(change_to_autonomous_client_));
  ASSERT_TRUE(spin_until(executor_, 500ms, [this] {
    return last_op_mode_ &&
           last_op_mode_->mode == OperationModeState::AUTONOMOUS;
  }));

  autoware_control_msgs::msg::Control cmd;
  cmd.longitudinal.velocity = 0.2f;
  control_pub_->publish(cmd);
  ASSERT_TRUE(
      spin_until(executor_, 500ms, [this] { return last_twist_.has_value(); }));
  EXPECT_NEAR(last_twist_->linear.x, 0.2, 1e-5);

  // Stop publishing control_cmd. After cmd_vel_timeout (0.2s) the watchdog
  // timer should publish a zero Twist.
  last_twist_.reset();
  ASSERT_TRUE(spin_until(executor_, 1500ms, [this] {
    return last_twist_.has_value() && std::abs(last_twist_->linear.x) < 1e-9 &&
           std::abs(last_twist_->angular.z) < 1e-9;
  }));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
