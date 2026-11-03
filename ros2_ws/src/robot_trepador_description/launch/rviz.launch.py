"""
RViz con el robot REAL: el modelo se orienta con la IMU y avanza con el PWM
que el robot reporta en /robot/estado.

  ros2 launch robot_trepador_description rviz.launch.py

Normalmente no se lanza solo: robot.launch.py (interfaz) lo incluye con rviz:=true.
Para poner la posición a cero:
  ros2 topic pub --once /robot/odom_reiniciar std_msgs/msg/Bool "{data: true}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('robot_trepador_description')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'descripcion.launch.py'))),
        Node(package='robot_trepador_description', executable='odometria_imu', output='screen'),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(share, 'rviz', 'robot.rviz')]),
    ])
