#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rclpy
import signal
import threading
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from math import copysign, sqrt, pow
from geometry_msgs.msg import Twist, Point
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class CalibrateLinear(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        signal.signal(signal.SIGINT, self.shutdown)

        # 파라미터 선언
        self.declare_parameter('test_distance', 1.0)          # 테스트할 거리 (m)
        self.declare_parameter('speed', 0.2)                  # 이동 속도 (m/s)
        self.declare_parameter('tolerance', 0.03)             # 허용 오차 (m)
        self.declare_parameter('odom_linear_scale_correction', 1.0)  # 오도메트리 보정 계수
        self.declare_parameter('start_test', False)           # 테스트 시작 여부
        self.update_param()
        
        # 프레임 ID
        self.odom_frame = 'odom'
        self.base_frame = 'base_footprint'

        # 속도 명령 퍼블리셔
        self.cmd_vel = self.create_publisher(Twist,"/controller/cmd_vel", 1)
       
        # TF(좌표 변환) 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.position = Point()
        self.x_start = self.position.x
        self.y_start = self.position.y    
        
        self.clock = self.get_clock()
        threading.Thread(target=self.main, daemon=True).start()
        
        self.get_logger().info('\033[1;32m%s\033[0m' % 'Bring up rqt_reconfigure to control the test')  

    def update_param(self):
        # 파라미터 업데이트
        self.start_test = self.get_parameter('start_test').value
        self.test_distance = self.get_parameter('test_distance').value
        self.speed = self.get_parameter('speed').value
        self.tolerance = self.get_parameter('tolerance').value
        self.odom_linear_scale_correction = self.get_parameter('odom_linear_scale_correction').value

    def main(self):
        # 메인 루프 (보정 테스트 수행)
        while True:
            self.update_param()
            move_cmd = Twist()
            if self.start_test:
                # odom 프레임과 base 프레임 사이의 현재 위치 가져오기
                self.position = self.get_position()

                # 시작점에서 현재까지의 유클리드 거리 계산
                original_distance = sqrt(pow((self.position.transform.translation.x - self.x_start), 2) +
                                pow((self.position.transform.translation.y - self.y_start), 2))

                # 보정 계수를 곱해 보정된 거리 계산
                calib_distance = original_distance * self.odom_linear_scale_correction

                # 목표 거리와의 차이(에러)
                error = calib_distance - self.test_distance
                # self.get_logger().info('\033[1;32moriginal:%f calib:%f\033[0m' % (original_distance, calib_distance))
                # 목표 거리에 도달했는지 확인
                if not self.start_test or abs(error) <  self.tolerance:
                    # 테스트 종료
                    self.start_test = rclpy.parameter.Parameter('start_test', rclpy.Parameter.Type.BOOL, False)
                    all_new_parameters = [self.start_test]
                    self.set_parameters(all_new_parameters)
                    self.get_logger().info('\033[1;32m%s\033[0m' % 'finish')
                else:
                    # 목표에 도달하지 않았다면, 에러 방향에 따라 직진/후진
                    move_cmd.linear.x = copysign(self.speed, -1 * error)
            else:
                # 테스트 시작 전에는 현재 위치를 시작점으로 기록
                self.position = self.get_position()
                if self.position is not None:
                    self.x_start = self.position.transform.translation.x
                    self.y_start = self.position.transform.translation.y

            # 속도 명령 퍼블리시
            self.cmd_vel.publish(move_cmd)
            time.sleep(0.05)

    def get_position(self):
        # odom과 base 프레임 사이의 현재 좌표 변환 가져오기
        try:
            trans = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1))
            return trans
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('\033[1;32m%s\033[0m' % 'TF Exception')
            return None

    def shutdown(self, signum, frame):
        # 노드 종료 시 로봇 정지
        self.get_logger().info('\033[1;32m%s\033[0m' % 'Stopping the robot...')
        self.start_test = False
        self.cmd_vel.publish(Twist())
        rclpy.shutdown()

def main():
    node = CalibrateLinear('calibrate_linear')
    rclpy.spin(node)  # ROS2 종료될 때까지 루프 대기

if __name__ == "__main__":
    main()
