# Copyright 2022 JhonDL
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_hospital-roboros')
    namespace = LaunchConfiguration('namespace')

    """ hace falta esto?
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')
    """

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/hospital_domain.pddl',
          'namespace': namespace
          }.items())
    

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch',
            'tb3_simulation_launch.py')),
        launch_arguments={
            'autostart': 'true',
            'params_file': os.path.join(example_dir, 'params', 'nav2_params.yaml')
        }.items())

    #specify the actions
    move_cmd = Node(
        package='plansys2-hospital-roboros',
        executable='move_action_node',
        name='move_action_node',
        output='screen',
        parameters=[])

    ld = LaunchDescription()

    #ld.add_action(declare_namespace_cmd)
    #esto para que es?

    # Declare the launch options
    ld.add_action(plansys2_cmd)
    ld.add_action(nav2_cmd)

    ld.add_action(move_cmd)

    return ld
