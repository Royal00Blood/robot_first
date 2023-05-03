import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    package_name = 'robot_first'

    rsp = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name),'launch','rsp.launch.py')]),launch_arguments={'use_sim_time':'true'}.items())
    
    pkg_path = os.path.join(get_package_share_directory('robot_first'))
    config = os.path.join(pkg_path, 'config', 'params.yaml' )
    world_path = os.path.join(pkg_path, 'world', "empty.world")
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join( get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')]),
        launch_arguments={"gui":"True",'extra_gazebo_args': '--ros-args --params-file ' + config,'"world_name"': world_path}.items())#,'extra_gazebo_args': '--ros-args --params-file ' + config
    
    
    declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
 
    
    spawn_entity = Node(package = 'gazebo_ros', 
                        executable = 'spawn_entity.py',
                        arguments = ['-topic','robot_description',
                                     '-entity','robot_first'],
                        output='screen')
    
    return LaunchDescription([rsp,gazebo,spawn_entity,])