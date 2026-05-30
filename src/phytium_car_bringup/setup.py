from setuptools import setup
import os
from glob import glob

package_name = 'phytium_car_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@phytium.com',
    description='飞腾派ROS2智能小车',
    license='MIT',
    entry_points={
        'console_scripts': [
            'd6_final = phytium_car_bringup.d6_final:main',
            'uart_bridge = phytium_car_bringup.uart_bridge:main',
        ],
    },
)