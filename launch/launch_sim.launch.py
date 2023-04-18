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
    
    config_file_name = 'params.yaml' 
    pkg_path = os.path.join(get_package_share_directory('robot_first'))
    config = os.path.join(pkg_path, 'config', config_file_name)
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')]),launch_arguments={"gui":"True",'extra_gazebo_args': '--ros-args --params-file ' + config}.items())#,'extra_gazebo_args': '--ros-args --params-file ' + config
    
    spawn_entity = Node(package = 'gazebo_ros', 
                        executable = 'spawn_entity.py',
                        arguments = ['-topic','robot_description',
                                     '-entity','robot_first'],
                        output='screen')
    
    return LaunchDescription([rsp,gazebo,spawn_entity,])


    