"""
Publica el modelo del robot (/robot_description) y sus TF fijas.

  ros2 launch robot_trepador_description descripcion.launch.py
  ros2 launch robot_trepador_description descripcion.launch.py sim:=true use_sim_time:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro = os.path.join(get_package_share_directory('robot_trepador_description'),
                         'urdf', 'robot_trepador.urdf.xacro')
    descripcion = ParameterValue(
        Command(['xacro ', xacro, ' sim:=', LaunchConfiguration('sim'),
                 ' rueda_x:=', LaunchConfiguration('rueda_x')]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='true = incluye los plugins de Gazebo'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rueda_x', default_value='0.2175',
                              description='Eje de las llantas desde el centro (m)'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': descripcion,
                          'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
