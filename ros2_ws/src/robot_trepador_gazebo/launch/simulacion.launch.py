"""
Simulación completa: Gazebo + robot + puente ROS-Gazebo + firmware simulado + RViz + interfaz.

  ros2 launch robot_trepador_gazebo simulacion.launch.py
  ros2 launch robot_trepador_gazebo simulacion.launch.py rviz:=false
  ros2 launch robot_trepador_gazebo simulacion.launch.py gazebo_gui:=false   # sin ventana de Gazebo
  ros2 launch robot_trepador_gazebo simulacion.launch.py empuje_max_n:=12.0
  ros2 launch robot_trepador_gazebo simulacion.launch.py bat_carga_inicial:=0.15   # probar avisos
  ros2 launch robot_trepador_gazebo simulacion.launch.py reparto_delantera:=0.0   # ver abajo

reparto_delantera: hoy los dos ESC van en paralelo (1.0 = mismo empuje en las dos hélices).
Con 0.0 la delantera se apaga y la trasera va al máximo: es el reparto que, según
claude/fisica_transiciones.md, deja levantar la nariz para pasar del suelo a la pared.

La interfaz es la misma que con el robot real: publica /robot/cmd_vel, /robot/esc y
/robot/parada, y lee /robot/imu y /robot/estado.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def lanzar_gazebo(context):
    mundo = LaunchConfiguration('mundo').perform(context)
    if not os.path.isabs(mundo):
        mundo = os.path.join(get_package_share_directory('robot_trepador_gazebo'), 'worlds', mundo)
    args = f'-r -v 2 {mundo}'
    if LaunchConfiguration('gazebo_gui').perform(context).lower() in ('false', '0'):
        args = '-s ' + args
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': args, 'on_exit_shutdown': 'true'}.items())]


def generate_launch_description():
    desc = get_package_share_directory('robot_trepador_description')
    gz = get_package_share_directory('robot_trepador_gazebo')
    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        DeclareLaunchArgument('mundo', default_value='habitacion.sdf',
                              description='Mundo de worlds/ o ruta absoluta a un .sdf'),
        DeclareLaunchArgument('gazebo_gui', default_value='true',
                              description='false = Gazebo sin ventana (solo servidor)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('interfaz', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.02'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('empuje_max_n', default_value='10.0',
                              description='Empuje de cada hélice a 1700 us, el máximo permitido (N)'),
        DeclareLaunchArgument('reparto_delantera', default_value='1.0',
                              description='Fracción del empuje en la hélice delantera. 1.0 = ESC '
                                          'en paralelo (firmware actual); 0.0 = solo la trasera'),
        DeclareLaunchArgument('bat_celdas', default_value='2', description='Celdas de la LiPo simulada'),
        DeclareLaunchArgument('bat_mah', default_value='2200.0', description='Capacidad (mAh)'),
        DeclareLaunchArgument('bat_carga_inicial', default_value='0.95', description='Carga al empezar (0..1)'),
        DeclareLaunchArgument('esc_max', default_value='1700',
                              description='Tope del botón + de la interfaz (us)'),

        # Gazebo encuentra el modelo del robot (package://...) sin configurar nada
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               os.path.dirname(desc) + os.pathsep +
                               os.environ.get('GZ_SIM_RESOURCE_PATH', '')),

        OpaqueFunction(function=lanzar_gazebo),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(desc, 'launch', 'descripcion.launch.py')),
            launch_arguments={'sim': 'true', 'use_sim_time': 'true'}.items()),

        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-topic', 'robot_description', '-name', 'escalador',
                        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                        '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')]),

        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(gz, 'config', 'puente_gz.yaml')}, sim_time]),

        Node(package='robot_trepador_gazebo', executable='firmware_simulado', output='screen',
             parameters=[{'empuje_max_n': LaunchConfiguration('empuje_max_n'),
                          'reparto_delantera': LaunchConfiguration('reparto_delantera'),
                          'bat_celdas': LaunchConfiguration('bat_celdas'),
                          'bat_mah': LaunchConfiguration('bat_mah'),
                          'bat_carga_inicial': LaunchConfiguration('bat_carga_inicial')}]),

        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(desc, 'rviz', 'robot.rviz')],
             parameters=[sim_time], condition=IfCondition(LaunchConfiguration('rviz'))),

        Node(package='robot_trepador_gui', executable='interfaz', output='screen',
             parameters=[{'esc_max': LaunchConfiguration('esc_max')}],
             condition=IfCondition(LaunchConfiguration('interfaz'))),
    ])
