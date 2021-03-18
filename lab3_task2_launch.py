#!/usr/bin/env python

# Copyright 1996-2021 Soft_illusion.
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

import os
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('webots_lab3_task2') #webots_lab3_task2
    core_dir = get_package_share_directory('webots_ros2_core')
    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(core_dir, 'launch', 'robot_launch.py')
        ),
        launch_arguments=[
            ('package', 'webots_lab3_task2'), #webots_lab3_task2
            ('executable', 'enable_robot'),
            ('world', PathJoinSubstitution( #change?
                [package_dir, 'worlds', 'lab3_task2.wbt'])), #lab3_task2.wbt
        ]
    )
    #change name of node when ready
    pole_looker = Node(
        package='webots_lab3_task2', #webots_lab3_task2
        executable='pole_looker', #lab3_task2     ?????
        name='master_node'
    )

    return LaunchDescription([
        webots, #KEEp, thats line 29 and down
        pole_looker #change to whatever line 41 is
    ])
