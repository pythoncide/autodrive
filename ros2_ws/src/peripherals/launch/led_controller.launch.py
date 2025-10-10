from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='peripherals',
            executable='led_controller',
            name='led_controller',
            parameters=[{'led_pin': 18}],   # ← GPIO 18번 사용
            output='screen'
        )
    ])