#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rclpy
import signal
import threading
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import Twist, Point
from math import copysign, pi, atan2, asin, degrees
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

def qua2rpy(quat):
    # 쿼터니언을 오일러 각(RPY)으로 변환 (ROS 형식의 Quaternion 사용)
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w

    roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = asin(2 * (w * y - x * z))
    yaw = atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))

    # 각도를 0~360도로 변환(convert the angle to the range of 0 to 360 degrees)
    if roll < 0:
        roll += 2 * pi
    
    if pitch < 0:
        pitch += 2 * pi
    
    if yaw < 0:
        yaw += 2 * pi
    
    return roll, pitch, yaw

def normalize_angle(angle):
    # 각도를 -pi ~ +pi 범위로 정규화
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
        
    return res

class CalibrateAngular(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        signal.signal(signal.SIGINT, self.shutdown)

        # 파라미터 선언
        self.declare_parameter('test_angle', 360.0)               # 회전할 목표 각도 (도 단위)
        self.declare_parameter('speed', 0.5)                      # 회전 속도 (rad/s)
        self.declare_parameter('tolerance', 1.5)                  # 허용 오차 (도 단위)
        self.declare_parameter('odom_angular_scale_correction', 1.0)  # 오도메트리 각도 보정 계수
        self.declare_parameter('start_test', False)               # 테스트 시작 여부
        self.update_param()
        
        # TF 프레임
        self.odom_frame = 'odom'
        self.base_frame = 'base_footprint'

        # 속도 퍼블리셔
        self.cmd_vel = self.create_publisher(Twist,"/controller/cmd_vel", 1)

        # TF 리스너 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.position = Point()
        self.x_start = self.position.x
        self.y_start = self.position.y    
        
        self.last_angle = 0        # 직전 각도
        self.reverse = 1           # 회전 방향 반전 플래그
        self.calib_angle = 0       # 누적 회전 각도
        
        self.clock = self.get_clock()
        threading.Thread(target=self.main, daemon=True).start()

        self.get_logger().info('\033[1;32m%s\033[0m' % 'Bring up rqt_reconfigure to control the test')  

    def update_param(self):
        # 파라미터 값 갱신
        self.start_test = self.get_parameter('start_test').value
        self.test_angle = self.get_parameter('test_angle').value
        self.speed = self.get_parameter('speed').value
        self.tolerance = self.get_parameter('tolerance').value
        self.odom_angular_scale_correction = self.get_parameter('odom_angular_scale_correction').value

    def main(self):
        # 메인 루프 (각도 보정 수행)
        while True:
            self.update_param()
            move_cmd = Twist()
            if self.start_test:
                # 현재 회전 각도 가져오기
                original_angle = degrees(self.get_odom_angle())
                self.test_angle = copysign(self.test_angle, self.reverse)

                # 직전 각도와 현재 각도의 차이를 보정 계수 적용 후 계산
                delta_angle = self.odom_angular_scale_correction * normalize_angle(original_angle - self.last_angle)

                # 누적 회전 각도 업데이트
                self.calib_angle += delta_angle

                # 목표 각도와의 오차 계산
                error = self.test_angle - self.calib_angle
                # self.get_logger().info('\033[1;32moriginal:%f calib:%f\033[0m' % (original_angle, self.calib_angle))

                self.last_angle = original_angle
                if abs(error) > self.tolerance and self.start_test:
                    # 목표 각도에 도달하지 못했다면 회전 명령 전송
                    move_cmd.angular.z = copysign(self.speed, error)
                else:
                    # 목표 각도에 도달했으면 초기화 및 종료
                    self.calib_angle = 0.0
                    self.start_test  = rclpy.parameter.Parameter('start_test', rclpy.Parameter.Type.BOOL, False)
                    all_new_parameters = [self.start_test]
                    self.set_parameters(all_new_parameters)
                    self.reverse = -self.reverse
                    self.last_angle = 0
                    self.get_logger().info('\033[1;32m%s\033[0m' % 'finish')

            self.cmd_vel.publish(move_cmd)
            time.sleep(0.05)
                
    def get_odom_angle(self):
        # Get the current transform between the odom and base frames
        try:
            trans = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1))
            return qua2rpy(trans.transform.rotation)[2]
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('\033[1;32m%s\033[0m' % 'TF Exception')
            return
        
    def shutdown(self, signum, frame):
        # 노드 종료 시 로봇 정지
        self.get_logger().info('\033[1;32m%s\033[0m' % 'Stopping the robot...')
        self.start_test = False
        self.cmd_vel.publish(Twist())
        rclpy.shutdown()

def main():
    node = CalibrateAngular('calibrate_angular')
    rclpy.spin(node)  # ROS2 종료될 때까지 대기(loop waiting for ROS2 to exit)

if __name__ == "__main__":
    main()
