from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='encoders_plotter',
            executable='SerialReader',  
            name='SerialReader'
        ),
        Node(
            package='plotjuggler',
            executable='plotjuggler',
            name='plotjuggler'
        )
    ])
