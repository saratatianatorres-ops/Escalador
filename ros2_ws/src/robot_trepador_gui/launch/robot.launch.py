"""
Arranca el agente micro-ROS (por el USB de la ESP puente) y la interfaz.

  ros2 launch robot_trepador_gui robot.launch.py
  ros2 launch robot_trepador_gui robot.launch.py puerto:=/dev/ttyACM0
  ros2 launch robot_trepador_gui robot.launch.py agente:=false   # si el agente ya corre aparte
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    puerto = LaunchConfiguration('puerto')
    return LaunchDescription([
        DeclareLaunchArgument('puerto', default_value='/dev/ttyUSB0',
                              description='Puerto USB de la ESP puente'),
        DeclareLaunchArgument('agente', default_value='true',
                              description='Lanzar también el micro_ros_agent'),
        DeclareLaunchArgument('esc_max', default_value='1800',
                              description='Tope del botón + de las hélices (us)'),

        Node(package='micro_ros_agent', executable='micro_ros_agent',
             arguments=['serial', '--dev', puerto, '-b', '921600'],
             output='screen', condition=IfCondition(LaunchConfiguration('agente'))),

        Node(package='robot_trepador_gui', executable='interfaz', output='screen',
             parameters=[{'esc_max': LaunchConfiguration('esc_max')}]),
    ])
