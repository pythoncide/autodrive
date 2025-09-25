# 로봇을 실행할 때 필요한 주요 노드와 하위 launch 파일들을 한꺼번에 띄우는 진입점
import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context):
    # 환경 변수 need_compile 값에 따라 빌드된 install/src 경로를 선택
    compiled = os.environ['need_compile']
    if compiled == 'True':  # 설치된 패키지 사용
        controller_package_path = get_package_share_directory('controller')
        app_package_path = get_package_share_directory('app')
        peripherals_package_path = get_package_share_directory('peripherals')
    else:   # 아직 빌드 안 했을 때 직접 src 경로 사용
        controller_package_path = '/home/ubuntu/ros2_ws/src/driver/controller'
        app_package_path = '/home/ubuntu/ros2_ws/src/app'
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'

    # 로봇 제어기 실행 (모터/제어 관련)
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/controller.launch.py')),
    )
    
    # depth 카메라 실행
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
    )

    # 라이다
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/lidar.launch.py')),
    )

    # ROS <-> WebSocket 브릿지(브라우저에서도 ROS 토픽 접근 가능)
    rosbridge_websocket_launch = ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
            output='screen'
        )

    # 카메라 영상 스트리밍을 웹에서 볼 수 있게 해줌
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        output='screen',
    )

    # 상위 애플리케이션 실행
    start_app_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/start_app.launch.py')),
    )

    # (네비게이션용) 초기 위치 설정
    init_pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(controller_package_path, 'launch/init_pose.launch.py')),
        launch_arguments={
            'namespace': '',  
            'use_namespace': 'false',
            'action_name': 'init',
        }.items(),
    )

    # 조이스틱 입력 기반 제어
    joystick_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(peripherals_package_path, 'launch/joystick_control.launch.py')),
    )

    startup_check_node = Node(
        package='bringup',
        executable='startup_check',
        output='screen',
    )

    # 센서, 제어기, 상위 앱, 초기화를 한번에 함
    return [
            startup_check_node,
            controller_launch,
            depth_camera_launch,
            lidar_launch,
            rosbridge_websocket_launch,
            web_video_server_node,
            start_app_launch,
            joystick_control_launch,
            init_pose_launch,
            ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
