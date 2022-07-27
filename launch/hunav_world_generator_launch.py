
# from os import path
# from os import environ
# from os import pathsep
import os, stat
from scripts import GazeboRosPaths
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.actions import LogInfo, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration, PythonExpression, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARGS = [
    DeclareLaunchArgument(
        'configuration_file', default_value='agents.yaml',
        description='Specify configuration file name in the cofig directory'
    ),
    DeclareLaunchArgument(
        'gazebo_world_file', default_value='empty_cafe.world',
        description='Specify the name of the base Gazebo world to be populated with pedestrians'
    ),
    DeclareLaunchArgument(
        'use_gazebo_obs', default_value='true',
        description='Whether to fill the agents obstacles with closest Gazebo obstacle or not'
    ),
    DeclareLaunchArgument(
        'update_rate', default_value='100.0',
        description='Update rate of the plugin'
    ),
    DeclareLaunchArgument(
        'robot_name', default_value='robot',
        description='Specify the name of the robot Gazebo model'
    ),
    DeclareLaunchArgument(
        'global_frame_to_publish', default_value='map',
        description='Name of the global frame in which the position of the agents are provided'
    ),
    DeclareLaunchArgument(
        'ignore_models', default_value='ground_plane cafe',
        description='list of Gazebo models that the agents should ignore as obstacles as the ground_plane. Indicate the models with a blank space between them'
    )
]

def generate_launch_description():

    world_file_name = LaunchConfiguration('gazebo_world_file')
    conf_file_name = LaunchConfiguration('configuration_file')
    gazebo_obs = LaunchConfiguration('use_gazebo_obs')
    upd_rate = LaunchConfiguration('update_rate')
    robot_name = LaunchConfiguration('robot_name')
    global_frame = LaunchConfiguration('global_frame_to_publish')
    ignore_models = LaunchConfiguration('ignore_models')


    # agent configuration file
    conf_file = PathJoinSubstitution([
        FindPackageShare('hunav_agent_manager'),
        'config',
        conf_file_name
    ])

    hunav_loader_node = Node(
        package='hunav_agent_manager',
        executable='hunav_loader',
        output='screen',
        parameters=[conf_file]
        #arguments=['--ros-args', '--params-file', conf_file]
    )

    # world base file
    world_file = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'worlds',
        world_file_name
    ])

    # No need to give permissions, so I comment this part
    # Give permissions to the world files so I can read and write
    # path = os.path.join(get_package_share_directory('hunav_gazebo_wrapper'),
    #             'worlds')
    # for root, dirs, files in os.walk(path):
    #     for w in files:
    #         fname = os.path.join(root, w)
    #         print("fname: ", fname)
    #         os.chmod(fname, stat.S_IRWXU) # | stat.S_IRWXO)


    # the node looks for the base_world file in the directory 'worlds'
    # of the package hunav_gazebo_plugin direclty. So we do not need to 
    # indicate the path
    hunav_gazebo_worldgen_node = Node(
        package='hunav_gazebo_wrapper',
        executable='hunav_gazebo_world_generator',
        output='screen',
        parameters=[{'base_world': world_file},
        {'use_gazebo_obs': gazebo_obs},
        {'update_rate': upd_rate},
        {'robot_name': robot_name},
        {'global_frame_to_publish': global_frame},
        {'ignore_models': ignore_models}]
        #arguments=['--ros-args', '--params-file', conf_file]
    )

    ordered_launch_event = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_loader_node,
            on_start=[
                LogInfo(msg='HunNavLoader started, launching HuNav_Gazebo_world_generator after 2 seconds...'),
                TimerAction(
                    period=2.0,
                    actions=[hunav_gazebo_worldgen_node],
                )
            ]
        )
    )


    ld = LaunchDescription(ARGS)

    #ld.add_action(permission)
    # Load the sim configuration file as ROS2 params
    ld.add_action(hunav_loader_node)
    # Read the sim configuration params from the hunav_loader node
    #ld.add_action(hunav_gazebo_worldgen_node)
    ld.add_action(ordered_launch_event)
    return ld
    

    




