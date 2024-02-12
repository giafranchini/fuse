#! /usr/bin/env python3

# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from launch_ros.actions import SetParameter, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_dir = FindPackageShare('fuse_tutorials')

    container_fuse = ComposableNodeContainer(
        name='container_fuse',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='fuse_optimizers',
                plugin='fuse_optimizers::FixedLagSmootherComponent',
                name='state_estimator',
                parameters=[PathJoinSubstitution([
                    pkg_dir, 'config', 'fuse_simple_tutorial.yaml'
                ])],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='both',
    )

    player = ExecuteProcess(
                cmd=['ros2', 'bag', 'play',
                    PathJoinSubstitution([pkg_dir, 'data', 'turtlebot3.bag']),
                    '--clock', '-l', '-d', '3'],
                output='screen'
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        player,
        container_fuse
    ])
