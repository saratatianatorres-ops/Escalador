#!/usr/bin/env bash
# Instala lo que necesita ros2_ws y compila. Sirve para:
#   - Ubuntu 24.04 + ROS 2 Jazzy  -> Gazebo Harmonic oficial (ros-jazzy-ros-gz)
#   - Ubuntu 22.04 + ROS 2 Humble -> Gazebo Harmonic desde packages.osrfoundation.org
#                                    (ros-humble-ros-gzharmonic)
# El micro_ros_agent y PlatformIO se instalan aparte: ver docs/instalacion_vm.md.
set -e

RAIZ="$(cd "$(dirname "$0")/.." && pwd)"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  DISTRO=jazzy
elif [ -f /opt/ros/humble/setup.bash ]; then
  DISTRO=humble
else
  echo "No encuentro ROS 2 Jazzy ni Humble en /opt/ros. Instálalo primero: docs/instalacion_vm.md, paso 4."
  exit 1
fi
echo "==> ROS 2 $DISTRO"

sudo apt update
sudo apt install -y python3-tk python3-colcon-common-extensions curl lsb-release gnupg \
  ros-$DISTRO-xacro ros-$DISTRO-robot-state-publisher ros-$DISTRO-rviz2 ros-$DISTRO-tf2-ros

if [ "$DISTRO" = jazzy ]; then
  sudo apt install -y ros-jazzy-ros-gz
else
  # Humble viene con Gazebo Fortress; el proyecto usa Gazebo Harmonic (plugins gz-sim-*).
  echo "==> Instalando Gazebo Harmonic para Humble"
  sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
  sudo apt update
  # ros-humble-ros-gzharmonic reemplaza a ros-humble-ros-gz (Fortress) si estaba instalado
  sudo apt install -y gz-harmonic ros-humble-ros-gzharmonic
fi

# El firmware del robot tiene que usar la misma distro que el PC
PIO="$RAIZ/firmware/robot_receptor/platformio.ini"
if ! grep -q "^board_microros_distro = $DISTRO" "$PIO"; then
  sed -i "s/^board_microros_distro = .*/board_microros_distro = $DISTRO/" "$PIO"
  echo "==> firmware/robot_receptor/platformio.ini ahora usa board_microros_distro = $DISTRO"
  echo "    Antes de compilar el firmware: cd firmware/robot_receptor && pio run -t clean_microros"
fi

source /opt/ros/$DISTRO/setup.bash
cd "$RAIZ/ros2_ws"
rm -rf build install log      # por si se compiló antes con otra distro
colcon build

echo
echo "Listo. En cada terminal nueva:  source $RAIZ/ros2_ws/install/setup.bash"
echo "Simulación:   ros2 launch robot_trepador_gazebo simulacion.launch.py"
echo "Robot real:   ros2 launch robot_trepador_gui robot.launch.py puerto:=/dev/ttyUSB0 rviz:=true"
