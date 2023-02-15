// Copyright 2023 Tier IV, Inc.
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
// limitations under the License

#include <chrono>
#include <memory>
#include "cargo_loading_service/cargo_loading_service.hpp"

namespace cargo_loading_service
{

CargoLoadingService::CargoLoadingService(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("cargo_loading_service", options),
  target_id_(""),
  approval_(false),
  aw_state_(in_parking_msgs::msg::InParkingStatus::AW_NONE)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  tier4_api_utils::ServiceProxyNodeInterface proxy(this);

  // Parameter
  cargo_loading_command_pub_hz_ = this->declare_parameter("cargo_loadging_command_pub_hz", 5.0);
  command_pub_sleep_time_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / cargo_loading_command_pub_hz_));

  // Callback group
  callback_group_service_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_subscription_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subscribe_option = rclcpp::SubscriptionOptions();
  subscribe_option.callback_group = callback_group_subscription_;

  // Service
  srv_cargo_loading_ = proxy.create_service<in_parking_msgs::srv::ExecuteInParkingTask>(
    "/parking/cargo_loading",
    std::bind(&CargoLoadingService::execCargoLoading, this, _1, _2),
      rmw_qos_profile_services_default, callback_group_service_);

  // Publisher
  pub_cargo_loading_state_ = this->create_publisher<v2i_interface_msgs::msg::InfrastructureCommandArray>(
    "/parking/cargo_loading_state", rclcpp::QoS{3}.transient_local());

  // Subscriber
  sub_parking_state_ =
    this->create_subscription<in_parking_msgs::msg::InParkingStatus>(
    "/in_parking/state", rclcpp::QoS{1},
    std::bind(&CargoLoadingService::onParkingState, this, std::placeholders::_1),
    subscribe_option);
  sub_cargo_loading_state_ =
    this->create_subscription<v2i_interface_msgs::msg::InfrastructureStateArray>(
    "/infrastructure_status", rclcpp::QoS{1},
    std::bind(&CargoLoadingService::onCargoLoadingState, this, std::placeholders::_1),
    subscribe_option);
}

void CargoLoadingService::execCargoLoading(
  const in_parking_msgs::srv::ExecuteInParkingTask::Request::SharedPtr request,
  const in_parking_msgs::srv::ExecuteInParkingTask::Response::SharedPtr response)
{
  using InfrastructureCommand = v2i_interface_msgs::msg::InfrastructureCommand;
  using InfrastructureCommandArray = v2i_interface_msgs::msg::InfrastructureCommandArray;
  using InParkingStatus = in_parking_msgs::msg::InParkingStatus;
  using ExecuteInParkingTaskResponse = in_parking_msgs::srv::ExecuteInParkingTask::Response;

  bool cmd_pub = true;
  int32_t aw_state;

  // Create command
  InfrastructureCommand command;
  command.type = "eva_beacon_system";
  command.id = request->value;

  target_id_ = request->value;

  {
    std::lock_guard<std::mutex> lock(mutex_cargo_loading_state_);
    approval_ = false;
  }

  // Publish command
  while(cmd_pub) {
    {
      std::lock_guard<std::mutex> lock(mutex_parking_state_);
      aw_state = aw_state_;
    }
    if (aw_state == InParkingStatus::AW_EMERGENCY) {
      command.state = cmd_error_;
    } else if (aw_state == InParkingStatus::AW_OUT_OF_PLACE) {
      command.state = InfrastructureCommand::SEND_ZERO;
    } else if (aw_state == InParkingStatus::AW_NONE) {
      response->state = ExecuteInParkingTaskResponse::FAIL;
      break;
    } else {
      command.state = cmd_requesting_;
    }
    InfrastructureCommandArray command_array;
    auto stamp = this->get_clock()->now();
    command.stamp = stamp;
    command_array.commands.push_back(command);
    command_array.stamp = stamp;
    pub_cargo_loading_state_->publish(command_array);
    rclcpp::sleep_for(command_pub_sleep_time_);
    {
      std::lock_guard<std::mutex> lock(mutex_cargo_loading_state_);
      if (approval_) {
        cmd_pub = false;
      }
    }
  }

  // Initialize class variables
  target_id_ = "";
  {
    std::lock_guard<std::mutex> lock(mutex_cargo_loading_state_);
    approval_ = false;
  }

  // Response
  response->state = ExecuteInParkingTaskResponse::SUCCESS;
}

void CargoLoadingService::onParkingState(
  const in_parking_msgs::msg::InParkingStatus::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_parking_state_);
    aw_state_ = msg->aw_state;
  }
  RCLCPP_DEBUG(this->get_logger(), "Subscribed /in_parking/state:%s",
               rosidl_generator_traits::to_yaml(*msg).c_str());
}

void CargoLoadingService::onCargoLoadingState(
  const v2i_interface_msgs::msg::InfrastructureStateArray::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_cargo_loading_state_);
    for (const auto & state : msg->states) {
      if (state.id.compare(target_id_) == 0) {
        approval_ = state.approval;
      }
    }
  }
  RCLCPP_DEBUG(this->get_logger(), "Subscribed /infrastructure_status:%s",
               rosidl_generator_traits::to_yaml(*msg).c_str());
}
}  // namespace cargo_loading_service

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(cargo_loading_service::CargoLoadingService)
