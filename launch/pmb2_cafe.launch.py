from os import path
from os import environ
from os import pathsep
from scripts import GazeboRosPaths
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable, 
                            DeclareLaunchArgument, ExecuteProcess, Shutdown, 
                            RegisterEventHandler, TimerAction, LogInfo, SetLaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration, 
                            PythonExpression, EnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from controller_manager.launch_utils import generate_load_controller_launch_description
from launch_pal.include_utils import include_scoped_launch_py_description

def generate_launch_description():

    #set_log_level = SetLaunchConfiguration('log_level', 'debug')
    

    # to activate the use of nvidia gpu
    use_nvidia_gpu = [
        '__NV_PRIME_RENDER_OFFLOAD=1 ',
        '__GLX_VENDOR_LIBRARY_NAME=nvidia ',
    ]

    # World generation parameters
    world_file_name = LaunchConfiguration('base_world')
    gz_obs = LaunchConfiguration('use_gazebo_obs')
    rate = LaunchConfiguration('update_rate')
    robot_name = LaunchConfiguration('robot_name')
    global_frame = LaunchConfiguration('global_frame_to_publish')
    use_navgoal = LaunchConfiguration('use_navgoal_to_start')
    navgoal_topic = LaunchConfiguration('navgoal_topic')
    ignore_models = LaunchConfiguration('ignore_models')
    navigation = LaunchConfiguration('navigation')

    # Robot parameters
    namespace = LaunchConfiguration('robot_namespace')
    scan_model = LaunchConfiguration('laser_model')
    use_rgbd = LaunchConfiguration('rgbd_sensors')
    gz_x = LaunchConfiguration('gzpose_x')
    gz_y = LaunchConfiguration('gzpose_y')
    gz_z = LaunchConfiguration('gzpose_z')
    gz_R = LaunchConfiguration('gzpose_R')
    gz_P = LaunchConfiguration('gzpose_P')
    gz_Y = LaunchConfiguration('gzpose_Y')


    # agent configuration file
    agent_conf_file = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'scenarios',
        LaunchConfiguration('configuration_file')
    ])

    # Read the yaml file and load the parameters
    hunav_loader_node = Node(
        package='hunav_agent_manager',
        executable='hunav_loader',
        output='screen',
        parameters=[agent_conf_file]
        #arguments=['--ros-args', '--params-file', conf_file]
    )

    # world base file
    world_file = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'worlds',
        world_file_name
    ])

    # the node looks for the base_world file in the directory 'worlds'
    # of the package hunav_gazebo_plugin directly. So we do not need to 
    # indicate the path
    hunav_gazebo_worldgen_node = Node(
        package='hunav_gazebo_wrapper',
        executable='hunav_gazebo_world_generator',
        output='screen',
        parameters=[{'base_world': world_file},
        {'use_gazebo_obs': gz_obs},
        {'update_rate': rate},
        {'robot_name': robot_name},
        {'global_frame_to_publish': global_frame},
        {'use_navgoal_to_start': use_navgoal},
        {'navgoal_topic': navgoal_topic},
        {'ignore_models': ignore_models}]
        #arguments=['--ros-args', '--params-file', conf_file]
    )

    ordered_launch_event = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_loader_node,
            on_start=[
                LogInfo(msg='HunNavLoader started, launching HuNav_Gazebo_world_generator after 3 seconds...'),
                TimerAction(
                    period=3.0,
                    actions=[hunav_gazebo_worldgen_node],
                )
            ]
        )
    )
    

    # Then, launch the generated world in Gazebo 
    my_gazebo_models = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'models',
    ])

    config_file_name = 'params.yaml' 
    pkg_dir = get_package_share_directory('hunav_gazebo_wrapper') 
    config_file = path.join(pkg_dir, 'launch', config_file_name) 

    
    model, plugin, media = GazeboRosPaths.get_paths()
    #print('model:', model)

    if 'GAZEBO_MODEL_PATH' in environ:
        model += pathsep+environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += pathsep+environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        media += pathsep+environ['GAZEBO_RESOURCE_PATH']

    env = {
        'GAZEBO_MODEL_PATH': model,
        'GAZEBO_PLUGIN_PATH': plugin,
        'GAZEBO_RESOURCE_PATH': media
    }
    print('env:', env)

    set_env_gazebo_model = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', 
        value=[EnvironmentVariable('GAZEBO_MODEL_PATH'), my_gazebo_models]
    )
    set_env_gazebo_resource = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH', 
        value=[EnvironmentVariable('GAZEBO_RESOURCE_PATH'), my_gazebo_models]
    )
    set_env_gazebo_plugin = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH', 
        value=[EnvironmentVariable('GAZEBO_PLUGIN_PATH'), plugin]
    )

    
    world_path = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'worlds',
        'generatedWorld.world' #'empty_cafe.world' #'pmb2_cafe.world'
    ])

    #print("World_path: ", world_path)

    gzserver_cmd = [
        use_nvidia_gpu,
        'gzserver ',
        #'--pause ',
        # Pass through arguments to gzserver
         world_path, 
        _boolean_command('verbose'), '',
        '-s ', 'libgazebo_ros_init.so',
        '-s ', 'libgazebo_ros_factory.so',
        #'-s ', 'libgazebo_ros_state.so',
        '--ros-args',
        '--params-file', config_file,
    ]

    gzclient_cmd = [
        use_nvidia_gpu,
        'gzclient',
        _boolean_command('verbose'), ' ',
    ]

    gzserver_process = ExecuteProcess(
        cmd=gzserver_cmd,
        output='screen',
        #additional_env=env,
        shell=True,
        on_exit=Shutdown(),
        #condition=IfCondition(LaunchConfiguration('server_required')),
    )

    gzclient_process = ExecuteProcess(
        cmd=gzclient_cmd,
        output='screen',
        #additional_env=env,
        shell=True,
        on_exit=Shutdown(),
        #condition=IfCondition(LaunchConfiguration('server_required')),
    )


    # hunav_manager node
    hunav_manager_node = Node(
        package='hunav_agent_manager',
        executable='hunav_agent_manager',
        name='hunav_agent_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    metrics_file = PathJoinSubstitution([
        FindPackageShare('hunav_evaluator'),
        'config',
        LaunchConfiguration('metrics_file')
    ])

    # hunav_evaluator node
    hunav_evaluator_node = Node(
        package='hunav_evaluator',
        executable='hunav_evaluator_node',
        output='screen',
        parameters=[metrics_file]
    )

    # DO NOT Launch this if any robot localization is launched
    static_tf_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        output='screen',
        arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        # other option: arguments = "0 0 0 0 0 0 pmb2 base_footprint".split(' ')
        condition=UnlessCondition(navigation)
    )


    # ----------------------------------------------------------
    # ----------------------------------------------------------
    #       PMB2 ROBOT
    # ----------------------------------------------------------
    # ----------------------------------------------------------
    pmb2_gazebo_launch = PathJoinSubstitution([
        FindPackageShare("hunav_gazebo_wrapper"),
        "launch",
        "pmb2_hunav.launch.py"
    ],)

    # map_path = PathJoinSubstitution([
    #     FindPackageShare("hunav_rviz2_panel"),
    #     "maps",
    #     "map_cafe2.yaml"
    # ],)
    map_dir = get_package_share_directory('hunav_gazebo_wrapper') 
    map_path = path.join(map_dir, 'maps', 'cafe.yaml') 

    pmb2_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pmb2_gazebo_launch]),
        launch_arguments={
            'robot_name':  robot_name,
            'laser_model':  'sick-571-gpu', #scan_model,
            'add_on_module': 'rgbd-sensors',
            'is_public_sim': 'True',
            'use_sim_time': 'True',
            'world_name': map_path,
            'x': gz_x,
            'y': gz_y,
            'yaw': gz_Y,
            'navigation': 'True',
            'advanced_navigation': 'False',
            'slam': 'False',
        }.items()
    )

    # pmb2_gazebo =include_scoped_launch_py_description(
    #     pkg_name='hunav_gazebo_wrapper', 
    #     paths=['launch', 'pmb2_hunav.launch.py'],
    #     launch_arguments={
    #         'robot_name': 'pmb2',  #robot_name,
    #         'laser_model': 'sick-571', #scan_model,
    #         'is_public_sim': 'True',
    #         'use_sim_time': 'True',
    #         'world_name': map_path,
    #         'x': '0.0', #gz_x,
    #         'y': '0.0', #gz_y,
    #         'yaw': '0.0', #gz_Y,
    #         'navigation': 'False', #navigation,
    #         'advanced_navigation': 'False',
    #         'slam': 'False',
    #     }
    # )

    # Do not launch Gazebo until the world has been generated
    gz_launch_event = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_gazebo_worldgen_node,
            on_start=[
                LogInfo(msg='GenerateWorld started, launching Gazebo after 3 seconds...'),
                TimerAction(
                    period=3.0,
                    actions=[gzserver_process, gzclient_process],
                )
            ]
        )
    )
    # gz_launch_event = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=hunav_gazebo_worldgen_node,
    #         on_exit=[
    #             LogInfo(msg='GenerateWorld finished, launching Gazebo & pmb2 robot in 2 seconds...'),
    #             TimerAction(
    #                 period=2.0,
    #                 actions=[pmb2_gazebo, gzserver_process, gzclient_process],
    #             )
    #         ]
    #     )
    # )


    declare_agents_conf_file = DeclareLaunchArgument(
        'configuration_file', default_value='agents_cafe.yaml',
        description='Specify configuration file name in the cofig directory'
    )
    declare_metrics_conf_file = DeclareLaunchArgument(
        'metrics_file', default_value='metrics.yaml',
        description='Specify the name of the metrics configuration file in the cofig directory'
    )
    declare_arg_world = DeclareLaunchArgument(
        'base_world', default_value='cafe.world',
        description='Specify world file name'
    )
    declare_gz_obs = DeclareLaunchArgument(
        'use_gazebo_obs', default_value='True',
        description='Whether to fill the agents obstacles with closest Gazebo obstacle or not'
    )
    declare_update_rate = DeclareLaunchArgument(
        'update_rate', default_value='100.0',
        description='Update rate of the plugin'
    )
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='pmb2',
        description='Specify the name of the robot Gazebo model'
    )
    declare_frame_to_publish = DeclareLaunchArgument(
        'global_frame_to_publish', default_value='map',
        description='Name of the global frame in which the position of the agents are provided'
    )
    declare_use_navgoal = DeclareLaunchArgument(
        'use_navgoal_to_start', default_value='False',
        description='Whether to start the agents movements when a navigation goal is received or not'
    )
    declare_navgoal_topic = DeclareLaunchArgument(
        'navgoal_topic', default_value='goal_pose',
        description='Name of the topic in which navigation goal for the robot will be published'
    )
    declare_navigation = DeclareLaunchArgument(
        'navigation', default_value='False',
        description='If launch the pmb2 navigation system'
    )
    declare_ignore_models = DeclareLaunchArgument(
        'ignore_models', default_value='ground_plane cafe',
        description='list of Gazebo models that the agents should ignore as obstacles as the ground_plane. Indicate the models with a blank space between them'
    )
    declare_arg_verbose = DeclareLaunchArgument(
        'verbose', default_value='False',
        description='Set "true" to increase messages written to terminal.'
    )
    declare_arg_namespace = DeclareLaunchArgument('robot_namespace', default_value='',
            description='The type of robot')
    #DeclareLaunchArgument('gzpose', default_value='-x 0.0 -y 0.0 -z 0.1 -R 0.0 -P 0.0 -Y 1.57',
    #                      description='The robot initial position in the world')
    declare_arg_px = DeclareLaunchArgument('gzpose_x', default_value='0.0',
            description='The robot initial position in the X axis of the world')
    declare_arg_py = DeclareLaunchArgument('gzpose_y', default_value='0.0',
            description='The robot initial position in the Y axis of the world')
    declare_arg_pz = DeclareLaunchArgument('gzpose_z', default_value='0.25',
            description='The robot initial position in the Z axis of the world')
    declare_arg_pR = DeclareLaunchArgument('gzpose_R', default_value='0.0',
            description='The robot initial roll angle in the world')
    declare_arg_pP = DeclareLaunchArgument('gzpose_P', default_value='0.0',
            description='The robot initial pitch angle in the world')
    declare_arg_pY = DeclareLaunchArgument('gzpose_Y', default_value='0.0',
            description='The robot initial yaw angle in the world')
    declare_arg_laser = DeclareLaunchArgument('laser_model', default_value='sick-571-gpu', 
            description='the laser model to be used')
    declare_arg_rgbd = DeclareLaunchArgument('rgbd_sensors', default_value='True',
            description='whether to use rgbd cameras or not')


    # Do not launch Gazebo until the world has been generated
    pmb2_launch_event = RegisterEventHandler(
        OnProcessExit(
            target_action=hunav_gazebo_worldgen_node,
            on_exit=[
                LogInfo(msg='GenerateWorld finished, launching pmb2 robot after 5 seconds...'),
                TimerAction(
                    period=5.0,
                    actions=[pmb2_gazebo],
                )
            ]
        )
    )
    
    

    ld = LaunchDescription()

    #ld.add_action(set_log_level)

    # set environment variables
    ld.add_action(set_env_gazebo_model)
    ld.add_action(set_env_gazebo_resource)
    ld.add_action(set_env_gazebo_plugin)

    # Declare the launch arguments
    ld.add_action(declare_agents_conf_file)
    ld.add_action(declare_metrics_conf_file)
    ld.add_action(declare_arg_world)
    ld.add_action(declare_gz_obs)
    ld.add_action(declare_update_rate)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_frame_to_publish)
    ld.add_action(declare_use_navgoal)
    ld.add_action(declare_navgoal_topic)
    ld.add_action(declare_navigation)
    ld.add_action(declare_ignore_models)
    ld.add_action(declare_arg_verbose)
    ld.add_action(declare_arg_namespace)
    ld.add_action(declare_arg_laser)
    ld.add_action(declare_arg_rgbd)
    ld.add_action(declare_arg_px)
    ld.add_action(declare_arg_py)
    ld.add_action(declare_arg_pz)
    ld.add_action(declare_arg_pR)
    ld.add_action(declare_arg_pP)
    ld.add_action(declare_arg_pY)

    # Generate the world with the agents
    # launch hunav_loader and the WorldGenerator
    # 2 seconds later
    ld.add_action(hunav_loader_node)
    ld.add_action(ordered_launch_event)

    # hunav behavior manager node
    ld.add_action(hunav_manager_node)
    # hunav evaluator
    ld.add_action(hunav_evaluator_node)

    # launch Gazebo after worldGenerator 
    ld.add_action(gz_launch_event)
    ld.add_action(static_tf_node)

    # spawn robot in Gazebo
    ld.add_action(pmb2_gazebo)
  


    return ld

    


# Add boolean commands if true
def _boolean_command(arg):
    cmd = ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd
