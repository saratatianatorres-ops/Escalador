from glob import glob
from setuptools import setup

paquete = 'robot_trepador_gui'

setup(
    name=paquete,
    version='0.1.0',
    packages=[paquete],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + paquete]),
        ('share/' + paquete, ['package.xml']),
        ('share/' + paquete + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Interfaz de control del robot trepador',
    license='MIT',
    entry_points={
        'console_scripts': [
            'interfaz = robot_trepador_gui.interfaz:main',
        ],
    },
)
