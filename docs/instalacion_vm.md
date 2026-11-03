# Instalación en una máquina virtual (Windows + VirtualBox)

Objetivo: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic + micro_ros_agent + PlatformIO, listo para
compilar los firmwares y lanzar la interfaz o la simulación.

## 1. Descargas (en Windows)

- VirtualBox 7.x para Windows y el **Extension Pack** de la misma versión: https://www.virtualbox.org/wiki/Downloads
- Ubuntu **24.04 LTS Desktop** (.iso, ~6 GB): https://ubuntu.com/download/desktop
- Driver USB de la ESP32 para Windows (CP210x de Silicon Labs o CH340, según la placa), para que Windows
  la reconozca y VirtualBox pueda pasarla.

Instala VirtualBox y luego haz doble clic en el Extension Pack.

## 2. Crear la máquina

Nueva → nombre `ros2-robot`, elige la .iso y **marca "Omitir instalación desatendida"**.

- RAM: 8192 MB (mínimo 6144) · CPU: 4 · Disco: 60 GB (dinámico)
- Pantalla: 128 MB de vídeo, controlador VMSVGA, **Habilitar aceleración 3D** (Gazebo y RViz la necesitan)
- USB: **Controlador USB 3.0 (xHCI)**; con la ESP32 conectada, botón "+" y elige el CP210x/CH340 para crear el filtro
- Red: NAT (por defecto)

Inicia la máquina e instala Ubuntu de forma normal ("Instalación normal", con actualizaciones).

## 3. Primeros pasos en Ubuntu

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential dkms linux-headers-$(uname -r) git curl
```

Menú Dispositivos → Insertar imagen de CD de las Guest Additions → ejecuta `autorun.sh` → reinicia
(pantalla ajustable y portapapeles). Activa Dispositivos → Portapapeles compartido → Bidireccional.

## 4. ROS 2 Jazzy

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

Prueba: `ros2 run demo_nodes_cpp talker` en una terminal y `ros2 run demo_nodes_py listener` en otra.

## 5. micro_ros_agent

```bash
sudo rosdep init; rosdep update
mkdir -p ~/microros_ws/src && cd ~/microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup   # jazzy o humble
rosdep install --from-paths src --ignore-src -y
colcon build && source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
echo "source ~/microros_ws/install/local_setup.bash" >> ~/.bashrc
```

Prueba: `ros2 run micro_ros_agent micro_ros_agent --help`

## 6. PlatformIO (compilar y subir firmwares)

```bash
curl -fsSL -o /tmp/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 /tmp/get-platformio.py
echo 'export PATH=$PATH:$HOME/.platformio/penv/bin' >> ~/.bashrc
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo udevadm control --reload-rules
sudo usermod -aG dialout $USER
sudo apt remove -y brltty      # roba los puertos CH340
```

Cierra sesión y vuelve a entrar (para el grupo dialout). Opcional: VS Code con la extensión PlatformIO IDE.

## 7. El proyecto

```bash
cd ~ && git clone https://github.com/saratatianatorres-ops/Escalador.git
cd Escalador && ./scripts/instalar_dependencias.sh     # Gazebo, RViz, xacro... y compila ros2_ws
echo "source ~/Escalador/ros2_ws/install/setup.bash" >> ~/.bashrc && source ~/.bashrc

cd firmware/robot_receptor && pio run -t upload        # 1.ª vez: 10-30 min (compila micro-ROS)
cd ../puente_pc && pio run -t upload
```

El repositorio es privado: al clonar, GitHub pide tu usuario y un *token* (Settings → Developer settings →
Personal access tokens) en lugar de la contraseña.

Al subir cada firmware, conecta solo esa ESP. Verifica el puerto con `ls /dev/ttyUSB* /dev/ttyACM*`.

## 8. Antes de las pruebas

Haz una **instantánea** (Máquina → Tomar instantánea) con todo funcionando, para volver ahí si algo se rompe.

## Problemas típicos

| Síntoma | Solución |
|---|---|
| No aparece /dev/ttyUSB0 | Filtro USB en VirtualBox; desconectar y conectar; `sudo apt remove brltty` |
| Permission denied en el puerto | `groups` debe mostrar dialout; cierra sesión |
| La ESP aparece en Windows y no en la VM | Menú Dispositivos → USB → marcar la ESP |
| Error de distro en el robot | `board_microros_distro = jazzy`; después de cambiarla, `pio run -t clean_microros` |
| El robot se reconecta a cada rato | Más CPU a la VM, cerrar programas en Windows |
| RViz o Gazebo en negro o se cierran | `export LIBGL_ALWAYS_SOFTWARE=1` antes de lanzar |
| Upload falla con "Failed to connect" | Mantén pulsado BOOT de la ESP al empezar la subida |
