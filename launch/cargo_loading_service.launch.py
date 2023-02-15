# Copyright 2023 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _create_node(node_name, class_name, **kwargs):
    return ComposableNode(
        namespace="cargo_loading",
        name=node_name,
        package="cargo_loading_service",
        plugin="cargo_loading_service::" + class_name,
        **kwargs
    )


def generate_launch_description():

    components = [
        _create_node('cargo_loading_service', 'CargoLoadingService')
    ]
    container = ComposableNodeContainer(
        namespace="cargo_loading",
        name="container_mt",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=components,
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log-level", default="info")],
    )
    return launch.LaunchDescription([container])
