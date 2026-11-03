#!/usr/bin/env bash
# Instala lo que necesita ros2_ws (Ubuntu 24.04 + ROS 2 Jazzy ya instalado) y compila.
# El micro_ros_agent y PlatformIO se instalan aparte: ver docs/instalacion_vm.md.
set -e

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "No encuentro ROS 2 Jazzy (/opt/ros/jazzy). Instálalo primero: docs/instalacion_vm.md, paso 4."
  exit 1
fi

sudo apt update
sudo apt install -y python3-tk python3-colcon-common-extensions \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-rviz2 \
  ros-jazzy-ros-gz ros-jazzy-tf2-ros

RAIZ="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
cd "$RAIZ/ros2_ws"
colcon build --symlink-install

echo
echo "Listo. En cada terminal nueva:  source $RAIZ/ros2_ws/install/setup.bash"
echo "Simulación:   ros2 launch robot_trepador_gazebo simulacion.launch.py"
echo "Robot real:   ros2 launch robot_trepador_gui robot.launch.py puerto:=/dev/ttyUSB0 rviz:=true"
