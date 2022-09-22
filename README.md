# hunav_gazebo_wrapper

A ROS2 wrapper to use the [HuNavSim](https://github.com/robotics-upo/hunav_sim) with the Gazebo Simulator (tested with ROS Foxy and Gazebo 11). 

## Wrapper functioning

The wrapper is based on two components:

* The *hunav_gazebo_world_generator* node. It is in charge of reading the simulation configuration parameters (human agents parameters) from the *hunav_loader* node, and the base Gazebo world file. With these parameters, the node modifies the base world file and includes the indicated human agents and the Gazebo plugin to control them (HuNavPlugin).

* The *HuNavPlugin* is a Gazebo plugin in charge of interacting with the HuNavSim to control the human movements. This interaction is performed through ROS2 services. 

![](https://github.com/robotics-upo/hunav_gazebo_wrapper/blob/master/media/images/hunav_gazebo_wrapper.png)

## Generator parameters

The *hunav_gazebo_world_generator* node has these parameters:

*  ```gazebo_world_file```. Name of the base Gazebo world file. This world should contain the static models (obstacles, rooms, furniture, etc). The human agents will be added automatically to this world. The world files must be placed inside the directory *worlds*.  
 

### Plugin params

The plugin parameters are read as ROS2 parameters also by the *hunav_gazebo_world_generator* node, which is in charge of adding the plugin to the Gazebo world file.

*  ```update_rate```. Update rate (Hz) of the plugin proccess.
*  ```robot_name```. Name of the Gazebo model corresponding to the robot that will be spawned.
*  ```use_gazebo_obs```. If True, the closest obstacle for each agent will be computed and added to the agent features (Default: True).
* ```global_frame_to_publish```. Coordinate frame in which the pedestrian positions are provided.
* ```ignore_models```. List of the Gazebo models than must be ignored by the pedestrians like the ground plane.

An example snippet of the HuNavPlugin which is automatically inserted in the world file:

```html
<plugin name="hunav_plugin" filename="libHuNavPlugin.so">
    <update_rate>100.0</update_rate>
    <robot_name>robot</robot_name>
    <use_gazebo_obs>True</use_gazebo_obs>
    <global_frame_to_publish>map</global_frame_to_publish>
    <ignore_models>
    	<model>cafe</model>
        <model>ground_plane</model>
    </ignore_models>
</plugin>
```

## Dependencies

* The HuNavSim: https://github.com/robotics-upo/hunav_sim
* The ROS2 package *gazebo_ros_pkgs* is also required: 
  ```sh 
  sudo apt install ros-foxy-gazebo-ros-pkgs 
  ```
* The gazebo models should be reacheable through the enviroment variable 'GAZEBO_MODEL_PATH'. You can source /usr/share/gazebo/setup.sh to stablish it.  

## Compilation

* This is a ROS2 package so it must be placed inside a ROS2 workspace and compiled through the regular colcon tool. 
```sh
colcon build --packages-select hunav_gazebo_wrapper
```

## Example launching

* Example of a Gazebo world of a café with a static actor (Actor3) playing the role of the robot:
```sh
ros2 launch hunav_gazebo_wrapper example_cafe.launch.py
```
* You can modify the hunav agents spawned by modifiying the agents' configuration file in:
  ```[your_workspace]/src/hunav_sim/hunav_agent_manager/config/agents.yaml```
     
* If you prefer to create the agents' configuration file by using the GUI, you can do it by launching:
```sh
ros2 launch hunav_rviz2_panel hunav_rviz2_launch.py
```

* If you want to use your robot instead, you can check the launch file *pmb2_cafe.launch.py*, and replace the pmb2 robot by yours and spawning it in Gazebo.
