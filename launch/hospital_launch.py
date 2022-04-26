# Copyright 2022 RoboRos
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tiago = get_package_share_directory('br2_tiago')
    nav = get_package_share_directory('br2_navigation')
    plansys_hospital = get_package_share_directory('plansys_hospital')

    setup = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tiago, 'launch', 'sim.launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav, 'launch', 'tiago_navigation.launch.py'))
        )
    ])

    hospital_launch = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(plansys_hospital, 'launch', 'plansys_try_launch.py'))
        )
    ])

    ld = LaunchDescription()
    ld.add_action(setup)
    ld.add_action(hospital_launch)
    return ld