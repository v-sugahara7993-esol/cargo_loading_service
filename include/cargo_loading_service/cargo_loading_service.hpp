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

#ifndef CARGO_LOADING_SERVICE__CARGO_LOADING_SERVICE_HPP_
#define CARGO_LOADING_SERVICE__CARGO_LOADING_SERVICE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "tier4_api_utils/tier4_api_utils.hpp"
#include "in_parking_msgs/srv/execute_in_parking_task.hpp"
#include "in_parking_msgs/msg/in_parking_status.hpp"
#include "v2i_interface_msgs/msg/infrastructure_command.hpp"
#include "v2i_interface_msgs/msg/infrastructure_command_array.hpp"
#include "v2i_interface_msgs/msg/infrastructure_state.hpp"
#include "v2i_interface_msgs/msg/infrastructure_state_array.hpp"

namespace cargo_loading_service
{

class CargoLoadingService : public rclcpp::Node
{
public:
  explicit CargoLoadingService(const rclcpp::NodeOptions & options);

private:
  using ExecuteInParkingTask = in_parking_msgs::srv::ExecuteInParkingTask;
  using InfrastructureCommandArray = v2i_interface_msgs::msg::InfrastructureCommandArray;
  using InParkingStatus = in_parking_msgs::msg::InParkingStatus;
  using InfrastructureStateArray = v2i_interface_msgs::msg::InfrastructureStateArray;

  const uint32_t cmd_requesting_ = 1;
  const uint32_t cmd_error_ = 2;

  std::mutex mutex_cargo_loading_state_;
  std::mutex mutex_parking_state_;
  std::string target_id_;
  bool approval_;
  int32_t aw_state_;
  std::chrono::nanoseconds command_pub_sleep_time_;
  double cargo_loading_command_pub_hz_;

  // Callback group
  rclcpp::CallbackGroup::SharedPtr callback_group_service_;
  rclcpp::CallbackGroup::SharedPtr callback_group_subscription_;

  // Service
  tier4_api_utils::Service<ExecuteInParkingTask>::SharedPtr srv_cargo_loading_;

  // Publisher
  rclcpp::Publisher<InfrastructureCommandArray>::SharedPtr pub_cargo_loading_state_;

  // Subscriber
  rclcpp::Subscription<InParkingStatus>::SharedPtr sub_parking_state_;
  rclcpp::Subscription<InfrastructureStateArray>::SharedPtr sub_cargo_loading_state_;

  // Callback
  void execCargoLoading(
    const ExecuteInParkingTask::Request::SharedPtr request,
    const ExecuteInParkingTask::Response::SharedPtr response);
  void onParkingState(const InParkingStatus::ConstSharedPtr msg_ptr);
  void onCargoLoadingState(const InfrastructureStateArray::ConstSharedPtr msg_ptr);
};

}  // namespace cargo_loading_service

#endif  // CARGO_LOADING_SERVICE__CARGO_LOADING_SERVICE_HPP_
