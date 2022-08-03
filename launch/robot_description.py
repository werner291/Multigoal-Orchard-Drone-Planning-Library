from launch import LaunchDescription
from launch_ros.actions import Node

import os

def generate_launch_description():

    print(os.getcwd())

    return LaunchDescription([
        Node(
            package='rviz2',
            namespace='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{
                'robot_description': open('test_robots/urdf/bot.urdf', 'r').read(),
                'robot_description_semantic': open('test_robots/config/aerial_manipulator_drone.srdf', 'r').read(),
                'planning_scene_topic': '/planning_scene'
            }]
        ),
    ])