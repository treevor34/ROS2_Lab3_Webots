# Copyright 1996-2021 Soft_illusion.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from glob import glob
from setuptools import setup

package_name = 'webots_lab3_task2'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', [
    'resource/' + package_name
]))
data_files.append(('share/' + package_name, [     
    'launch/lab3_task2_launch.py' #lab3_launch.py SOOON
]))
data_files.append(('share/' + package_name + '/worlds', [  #lab3_task2.wbt
    'worlds/lab3_task2.wbt'
]))
data_files.append(('share/' + package_name + '/protos', [ #get rid of? 
    'protos/Robot_sense.proto'
]))
#data_files.append(
#    ('share/' + package_name + '/protos/icons', glob('protos/icons/*'))) #get rid of?
#data_files.append(
#    ('share/' + package_name + '/worlds/textures', glob('worlds/textures/*'))) #get rid of?
#data_files.append(
#    ('share/' + package_name + '/protos/textures', glob('protos/textures/*'))) #get rid of?

data_files.append(('share/' + package_name, [
    'package.xml'
]))

setup(
    name=package_name,
    version='1.0.5',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    keywords=['ROS2', 'Webots', 'Soft_Illusion',  #wtf are these lol
              'Tutorials', 'Youtube', 'Simulation'],
    maintainer='Soft_illusion',
    maintainer_email='harsh.b.kakashaniya@gmail.com', #nah not mine
    description='Projects for videos for webots ros2 tutorial series on youtube',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [  #do we keep enable_robot? defo change linefollower
            'enable_robot = webots_lab3_task2.slave:main',  #webots_lab3_task2.slave:main
            'pole_looker = webots_lab3_task2.master:main'  #webots_lab3_task2.master:main
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
