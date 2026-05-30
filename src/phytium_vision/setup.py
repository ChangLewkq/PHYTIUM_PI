from setuptools import setup
import os
from glob import glob

package_name = 'phytium_vision'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts_rtx3050'), glob('scripts_rtx3050/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@phytium.com',
    description='Clean Flyt-Pi vision pipeline: RGB cloud inference + local depth following',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rgb_sender = phytium_vision.rgb_sender:main',
            'target_depth_follower = phytium_vision.target_depth_follower:main',
            'cmd_vel_mux = phytium_vision.cmd_vel_mux:main',
            'safety_guard = phytium_vision.safety_guard:main',
        ],
    },
)
