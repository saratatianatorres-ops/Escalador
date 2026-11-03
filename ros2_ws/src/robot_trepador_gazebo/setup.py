from glob import glob
from setuptools import setup

paquete = 'robot_trepador_gazebo'

setup(
    name=paquete,
    version='0.1.0',
    packages=[paquete],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + paquete]),
        ('share/' + paquete, ['package.xml']),
        ('share/' + paquete + '/launch', glob('launch/*.py')),
        ('share/' + paquete + '/worlds', glob('worlds/*')),
        ('share/' + paquete + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Simulación del robot trepador en Gazebo',
    license='MIT',
    entry_points={
        'console_scripts': [
            'firmware_simulado = robot_trepador_gazebo.firmware_simulado:main',
        ],
    },
)
