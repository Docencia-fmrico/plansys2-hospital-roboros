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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys_hospital')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/hospital_domain.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions
    move_cmd = Node(
        package='plansys_hospital',
        executable='move_action_node',
        name='plansys_hospital',
        namespace=namespace,
        output='screen',
        parameters=[])
    '''
    move_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move',
        namespace=namespace,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'action_name': 'move',
                'publisher_port': 1668,
                'server_port': 1669,
                'server_timeout':50,
                'bt_xml_file': example_dir + '/bt_xml/move.xml'
            }
        ])
    '''
    pick_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='pick',
        namespace=namespace,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'action_name': 'pick',
                'bt_xml_file': example_dir + '/bt_xml/pick.xml'
            }
        ])
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    # Declare the launch options
    ld.add_action(plansys2_cmd)
    ld.add_action(move_cmd)
    ld.add_action(pick_cmd)

    return ld
