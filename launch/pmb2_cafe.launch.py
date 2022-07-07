from os import path
from os import environ
from os import pathsep
from scripts import GazeboRosPaths
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration, PythonExpression, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    namespace = LaunchConfiguration('robot_namespace')
    scan_model = LaunchConfiguration('laser_model')
    use_rgbd = LaunchConfiguration('rgbd_sensors')
    gz_x = LaunchConfiguration('gzpose_x')
    gz_y = LaunchConfiguration('gzpose_y')
    gz_z = LaunchConfiguration('gzpose_z')
    gz_R = LaunchConfiguration('gzpose_R')
    gz_P = LaunchConfiguration('gzpose_P')
    gz_Y = LaunchConfiguration('gzpose_Y')

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
        'empty_cafe.world' #'pmb2_cafe.world'
    ])

    gzserver_cmd = [
        'gzserver ',
        '--pause ',
        # Pass through arguments to gzserver
        LaunchConfiguration('world'), world_path, 
        _boolean_command('verbose'), '',
        '-s ', 'libgazebo_ros_init.so',
        '-s ', 'libgazebo_ros_factory.so',
        #'-s ', #'libgazebo_ros_state.so',
        '--ros-args',
        '--params-file', config_file,
    ]

    gzclient_cmd = [
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


    gazebo_spawn = PathJoinSubstitution(
        [FindPackageShare("pmb2_gazebo"),
        "launch",
        "pmb2_spawn.launch.py"],
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_spawn]),
        launch_arguments={'robot_namespace': namespace,
                        'laser_model': scan_model, 
                        'rgbd_sensors': use_rgbd,
                        'gzpose_x': gz_x,
                        'gzpose_y': gz_y,
                        'gzpose_Y': gz_Y}.items(),
    )


    human_nav_manager_node = Node(
        package='hunav_agent_manager',
        executable='hunav_agent_manager',
        name='hunav_agent_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # DO NOT Launch this if any robot localization is launched
    static_tf_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        output='screen',
        arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom']
        # other option: arguments = "0 0 0 0 0 0 pmb2 base_footprint".split(' ')
    )

    declare_arg_world = DeclareLaunchArgument(
        'world', default_value='',
        description='Specify world file name'
    )
    declare_arg_verbose = DeclareLaunchArgument(
        'verbose', default_value='true',
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
    declare_arg_laser = DeclareLaunchArgument('laser_model', default_value='sick-571',
            description='the laser model to be used')
    declare_arg_rgbd = DeclareLaunchArgument('rgbd_sensors', default_value='false',
            description='whether to use rgbd cameras or not')

    

    ld = LaunchDescription()

    # set environment variables
    ld.add_action(set_env_gazebo_model)
    ld.add_action(set_env_gazebo_resource)
    ld.add_action(set_env_gazebo_plugin)

    # Declare the launch arguments
    ld.add_action(declare_arg_world)
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

    # human nav behaviors node
    #ld.add_action(human_nav_manager_node)

    # launch Gazebo
    ld.add_action(gzserver_process)
    ld.add_action(gzclient_process)
    # spawn robot in Gazebo
    ld.add_action(spawn_robot)

    return ld

    


# Add boolean commands if true
def _boolean_command(arg):
    cmd = ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd
