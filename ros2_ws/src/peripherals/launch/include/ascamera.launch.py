from launch import LaunchDescription
import launch_ros.actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace="ascamera",
            package='ascamera',
            executable='ascamera_node',
            respawn=True,
            output='both',
            parameters=[
                {"usb_bus_no": -1},
                {"usb_path": "null"},
                {"confiPath": "/home/ubuntu/third_party_ros2/third_party_ws/src/ascamera/configurationfiles"},
                {"color_pcl": True},
                {"pub_tfTree": True},
                {"depth_width": 640},
                {"depth_height": 480},
                {"rgb_width": 640},
                {"rgb_height": 480},
                {"fps": 15},
            ]

        ),
      ])