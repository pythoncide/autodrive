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
                'rs': 21, 'e': 20, 'd4': 16, 'd5': 12, 'd6': 25, 'd7': 23,
                'width': 16,
                'topic': 'ui/lcd',
                'gpiochip': 0,  # /dev/gpiochip0
            }],
        ),
    ])