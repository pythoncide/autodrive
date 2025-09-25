# 앱 기능(라인트레이싱, 객체추적 등) 실행
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context):
    # 경로 설정
    compiled = os.environ['need_compile']
    if compiled == 'True':
        app_package_path = get_package_share_directory('app')
    else:
        app_package_path = '/home/ubuntu/ros2_ws/src/app'

    lidar_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/lidar_node.launch.py')),
    )

    line_following_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/line_following_node.launch.py')),
    )

    object_tracking_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/object_tracking_node.launch.py')),
    )

    ar_app_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/ar_app_node.launch.py')),
    )

    hand_gesture_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/hand_gesture_node.launch.py')),
    )

    return [lidar_node_launch,
            line_following_node_launch,
            object_tracking_node_launch,
            #ar_app_node_launch,
            hand_gesture_node_launch
            ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # LaunchDescription 객체 생성
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
