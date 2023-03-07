# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
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
from os import environ, pathsep

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_pal.include_utils import include_launch_py_description


def generate_launch_description():

    navigation_arg = DeclareLaunchArgument(
        'navigation', default_value='false',
        description='Specify if launching Navigation2'
    )

    pmb2_spawn = include_launch_py_description(
        'pmb2_gazebo', ['launch', 'pmb2_spawn.launch.py'])
    pmb2_bringup = include_launch_py_description(
        'pmb2_bringup', ['launch', 'pmb2_bringup.launch.py'])

    navigation = include_launch_py_description(
        'pmb2_2dnav', ['launch', 'pmb2_nav_bringup.launch.py'],
        condition=IfCondition(LaunchConfiguration('navigation')))

    pkg_path = get_package_prefix('pmb2_description')
    model_path = os.path.join(pkg_path, 'share')
    resource_path = pkg_path

    if 'GAZEBO_MODEL_PATH' in environ:
        model_path += pathsep + environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        resource_path += pathsep + environ['GAZEBO_RESOURCE_PATH']

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path))

    ld.add_action(pmb2_spawn)
    ld.add_action(pmb2_bringup)

    ld.add_action(navigation_arg)
    ld.add_action(navigation)

    return ld
