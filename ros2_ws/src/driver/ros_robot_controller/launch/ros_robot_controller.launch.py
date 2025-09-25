# 하드웨어 제어 패키지를 실행하기 위한 최소 설정 런치 스크립트.
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    imu_frame = LaunchConfiguration('imu_frame', default='imu_link')
    imu_frame_arg = DeclareLaunchArgument('imu_frame', default_value=imu_frame)

    # 하드웨어 제어 노드를 실행하면서, IMU의 프레임 이름을 설정
    ros_robot_controller_node = Node(
        package='ros_robot_controller',
        executable='ros_robot_controller',
        output='screen',    # 실행 로그를 터미널에 바로 출력
        parameters=[{'imu_frame': imu_frame}]
    )

    return LaunchDescription([
        imu_frame_arg,
        ros_robot_controller_node
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
