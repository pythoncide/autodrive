from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='peripherals',
            executable='lcd_controller',
            name='lcd_controller',
            output='screen',
            parameters=[{
                'rs': 26, 'e': 19, 'd4': 13, 'd5': 6, 'd6': 5, 'd7': 11,
                'width': 16,
                'topic': 'ui/lcd',
                'gpiochip': 0,  # /dev/gpiochip0
            }],
        ),
    ])