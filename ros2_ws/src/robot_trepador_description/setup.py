from glob import glob
from setuptools import setup

paquete = 'robot_trepador_description'

setup(
    name=paquete,
    version='0.1.0',
    packages=[paquete],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + paquete]),
        ('share/' + paquete, ['package.xml']),
        ('share/' + paquete + '/launch', glob('launch/*.py')),
        ('share/' + paquete + '/urdf', glob('urdf/*')),
        ('share/' + paquete + '/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Modelo URDF del robot trepador y visualización en RViz',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odometria_imu = robot_trepador_description.odometria_imu:main',
        ],
    },
)
