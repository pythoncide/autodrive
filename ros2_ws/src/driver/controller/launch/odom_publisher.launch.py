# 오도메트리 퍼블리셔 노드를 실행하는 런치 파일
# 로봇의 TF 구조 + 하드웨어 제어 + 오도메트리 퍼블리시가 동시에 준비
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction

def generate_launch_description():
    compiled = os.environ['need_compile']
    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace', default='false')
    odom_frame = LaunchConfiguration('odom_frame', default='odom')  # 오도메트리 좌표계 이름
    base_frame = LaunchConfiguration('base_frame', default='base_footprint')    # 로봇 본체 좌표계
    imu_frame = LaunchConfiguration('imu_frame', default='imu_link')    # IMU 센서 좌표계
    frame_prefix = LaunchConfiguration('frame_prefix', default='')  # 프레임 앞에 붙일 접두사

    namespace_arg = DeclareLaunchArgument('namespace', default_value=namespace)
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value=use_namespace)
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value=odom_frame)
    base_frame_arg = DeclareLaunchArgument('base_frame', default_value=base_frame)
    imu_frame_arg = DeclareLaunchArgument('imu_frame', default_value=imu_frame)
    frame_prefix_arg = DeclareLaunchArgument('frame_prefix', default_value=frame_prefix)

    # 패키지 경로 설정
    if compiled == 'True':
        rosmentor_description_package_path = get_package_share_directory('mentorpi_description')
        robot_controller_package_path = get_package_share_directory('ros_robot_controller')
        controller_package_path = get_package_share_directory('controller')
    else:
        rosmentor_description_package_path = '/home/ubuntu/ros2_ws/src/simulations/mentorpi_description'
        robot_controller_package_path = '/home/ubuntu/ros2_ws/src/driver/ros_robot_controller'
        controller_package_path = '/home/ubuntu/ros2_ws/src/driver/controller'

    # 로봇의 좌표계(TF) 구조를 정의
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(rosmentor_description_package_path, 'launch/robot_description.launch.py')
        ]),
        launch_arguments={
            'frame_prefix': frame_prefix,
            'use_gui': 'false',
            'use_rviz': 'false',
            'use_sim_time': 'false',
            'use_namespace': use_namespace,
            'namespace': namespace,
        }.items()
    )

    # 센서와 로봇 하드웨어 제어 연결
    robot_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(robot_controller_package_path, 'launch/ros_robot_controller.launch.py')
        ]),
        launch_arguments={
            'imu_frame': imu_frame,
        }.items()
    )

    odom_publisher_node = Node(
        package='controller',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
        parameters=[os.path.join(controller_package_path, 'config/calibrate_params.yaml'), {
            'base_frame_id': base_frame, 
            'odom_frame_id': odom_frame,
            'pub_odom_topic': True,
            }],  
    )

    return LaunchDescription([
        namespace_arg,
        use_namespace_arg,
        odom_frame_arg,
        base_frame_arg,
        imu_frame_arg,
        frame_prefix_arg,
        robot_description_launch,
        robot_controller_launch,
        odom_publisher_node
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
