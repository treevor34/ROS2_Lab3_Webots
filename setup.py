from glob import glob
from setuptools import setup

package_name = 'webots_lab3_task2'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', [
    'resource/' + package_name
]))
data_files.append(('share/' + package_name, [     
    'launch/lab3_task2_launch.py' 
]))
data_files.append(('share/' + package_name + '/worlds', [  #world
    'worlds/lab3_task2.wbt'#'worlds/WorldMultipleCylinders.wbt'
]))
#data_files.append(('share/' + package_name + '/protos', [ #get rid of? 
#    'protos/Robot_sense.proto'
#]))
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
    keywords=['ROS2', 'Webots', 'Trevor',  #wtf are these lol
              'Tutorials', 'Youtube', 'Simulation'],
    maintainer='Trevor',
    maintainer_email='tmfournier@usf.edu', 
    description='Project based off of SoftIllusion code',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            'enable_robot = webots_lab3_task2.slave:main',  #publisher file
            'pole_looker = webots_lab3_task2.master:main'  #subscriber file with logic
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
