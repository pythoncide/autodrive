from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 로봇 하위 시스템 - 주행 제어(Chassis control)
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('controller'),
            '/launch/controller.launch.py'
        ])
    )

    # 각속도 보정(Angular velocity calibration)
    calibrate_angular_node = Node(
        package='calibration',
        executable='calibrate_angular',
        output='screen'
    )

    # rqt_reconfigure GUI 실행 (실시간 파라미터 조정용)
    calibrate_rqt_reconfigure_node = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='calibrate_rqt_reconfigure'
    )

    return LaunchDescription([
        controller_launch,
        calibrate_angular_node,
        calibrate_rqt_reconfigure_node
    ])

if __name__ == '__main__':
    generate_launch_description()