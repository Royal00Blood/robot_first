from http.server import executable
import os

from ament_index_python.packages import get_packages_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDiscription
from launche.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from symbol import argument

def generate_launch_description():
    package_name = 'robot_first'
    rsp = IncludeLaunchDiscription(PythonLaunchDescriptionSource([os.path.join(get_packages_share_directory(package_name),'launch','rsp.launch.py')]),launch_arguments={'use_sim_time':'true'}.items())
    gazebo = IncludeLaunchDiscription(PythonLaunchDescriptionSource([os.path.join(get_packages_share_directory('gazebo_ros'),'launch','gazebo.launch.py')]),)
    spawn_entity = Node(package = 'gazebo_ros', executable = 'spawn_entity.py',
                        arguments = ['-topic','robot_description',
                                     '-entity','robot_first'],
                        output='screen')
    return LaunchDescription([rsp,gazebo,spawn_entity,])