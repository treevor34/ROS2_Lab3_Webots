import os
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('webots_lab3_task2') #webots_lab3_task2
    core_dir = get_package_share_directory('webots_ros2_core')
    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(core_dir, 'launch', 'robot_launch.py')
        ),
        launch_arguments=[
            ('package', 'webots_lab3_task2'), #webots_lab3_task2
            ('executable', 'enable_robot'),
            ('world', PathJoinSubstitution( 
                [package_dir, 'worlds', 'lab3_task2.wbt'])), #'WorldMultipleCylinders.wbt']))
        ]
    )
    
    pole_looker = Node(
        package='webots_lab3_task2', #webots_lab3_task2
        executable='pole_looker', #lab3_task2     ?????
        name='master_node'
    )

    return LaunchDescription([
        webots, #KEEp, thats line 29 and down
        pole_looker #change to whatever line 41 is
    ])
