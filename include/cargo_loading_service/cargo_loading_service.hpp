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

#include <string>

namespace cargo_loading_service
{

class CargoLoadingService : public rclcpp::Node
{
public:
  explicit CargoLoadingService(const rclcpp::NodeOptions & options);

private:
  using ExecuteInParkingTask = in_parking_msgs::srv::ExecuteInParkingTask;
  using InfrastructureCommand = v2i_interface_msgs::msg::InfrastructureCommand;
  using InfrastructureCommandArray = v2i_interface_msgs::msg::InfrastructureCommandArray;
  using InfrastructureStateArray = v2i_interface_msgs::msg::InfrastructureStateArray;
  using InParkingStatus = in_parking_msgs::msg::InParkingStatus;


  // constants
  enum class CMD_STATE : uint8_t
  {
    REQUESTING = 0b01,
    ERROR = 0b10
  };

  static constexpr char CMD_TYPE[] = "eva_beacon_system";

  // variable
  std::string facility_id_;
  int32_t aw_state_{InParkingStatus::NONE};
  double command_pub_hz_;
  bool finalize_;

  // Callback group
  rclcpp::CallbackGroup::SharedPtr callback_group_service_;
  rclcpp::CallbackGroup::SharedPtr callback_group_subscription_;

  // Service
  tier4_api_utils::Service<ExecuteInParkingTask>::SharedPtr srv_cargo_loading_;

  // Publisher
  rclcpp::Publisher<InfrastructureCommandArray>::SharedPtr pub_cargo_loading_state_;

  // Subscriber
  rclcpp::Subscription<InParkingStatus>::SharedPtr sub_inparking_status_;
  rclcpp::Subscription<InfrastructureStateArray>::SharedPtr sub_cargo_loading_state_;

  // Callback
  void execCargoLoading(
    const ExecuteInParkingTask::Request::SharedPtr request,
    const ExecuteInParkingTask::Response::SharedPtr response);
  void onInParkingState(const InParkingStatus::ConstSharedPtr msg_ptr);
  void onCargoLoadingState(const InfrastructureStateArray::ConstSharedPtr msg_ptr);

  // Function
  void InitParam();
  uint8_t getCommandState();
};

}  // namespace cargo_loading_service

#endif  // CARGO_LOADING_SERVICE__CARGO_LOADING_SERVICE_HPP_
