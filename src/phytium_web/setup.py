from setuptools import setup
import os
from glob import glob
package_name='phytium_web'
setup(name=package_name,version='2.0.0',packages=[package_name],package_data={package_name:['templates/*.html','static/*.css','static/*.js']},include_package_data=True,data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),('share/'+package_name,['package.xml']),(os.path.join('share',package_name,'launch'),glob('launch/*.py')),(os.path.join('share',package_name,'config'),glob('config/*.yaml'))],install_requires=['setuptools','flask'],zip_safe=True,maintainer='user',maintainer_email='user@phytium.com',description='Final Phytium ROS2 robot web dashboard',license='MIT',entry_points={'console_scripts':['web_server = phytium_web.app:main']})
