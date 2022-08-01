from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Pa
    ])

<launch>
    <param name="robot_description" textfile="$(dirname)/../test_robots/urdf/bot.urdf" />
    <param name="robot_description_semantic" textfile="$(dirname)/../test_robots/config/aerial_manipulator_drone.srdf" />
</launch>