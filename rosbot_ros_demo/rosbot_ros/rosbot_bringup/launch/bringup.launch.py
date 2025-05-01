# Copyright 2024 Husarion sp. z o.o.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim = LaunchConfiguration("use_sim", default="False")

    declare_healthcheck_arg = DeclareLaunchArgument(
        "healthcheck",
        default_value="False",
        description="Check if all node are up and ready, if not emit shutdown signal.",
        choices=["True", "true", "False", "false"],
    )

    declare_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="False",
        description="Launch RViz2 with the specified configuration file.",
        choices=["True", "true", "False", "false"],
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Namespace for all topics and tfs",
    )

    rosbot_bringup = FindPackageShare("rosbot_bringup")
    rosbot_controller = FindPackageShare("rosbot_controller")

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rosbot_controller, "launch", "controller.launch.py"])
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim": use_sim,
        }.items(),
    )

    healthcheck_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rosbot_bringup, "launch", "healthcheck.launch.py"])
        )
    )

    ekf_config = PathJoinSubstitution([rosbot_bringup, "config", "ekf.yaml"])

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        output="screen",
        parameters=[ekf_config],
        remappings=[
            ("/diagnostics", "diagnostics"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        namespace=namespace,
    )

    rviz_config = PathJoinSubstitution([rosbot_bringup, "config", "rviz_config.rviz"])

    rviz_node = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        cmd=['rviz2', '-d', rviz_config],
        output='screen'
    )

    laser_filter_config = PathJoinSubstitution([rosbot_bringup, "config", "laser_filter.yaml"])

    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[laser_filter_config],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        namespace=namespace,
    )

    actions = [
        declare_healthcheck_arg,
        declare_rviz_arg,
        declare_namespace_arg,
        controller_launch,
        healthcheck_launch,
        laser_filter_node,
        robot_localization_node,
        rviz_node,
    ]

    return LaunchDescription(actions)
