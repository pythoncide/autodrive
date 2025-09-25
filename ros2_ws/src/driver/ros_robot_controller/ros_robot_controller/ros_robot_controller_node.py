#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2023/08/28
# stm32 ros2 package

# 로봇 몸체와 ROS2 소프트웨어 사이 브리지 역할

import math
import time
import rclpy
import signal
import threading
import yaml  # 已导入 PyYAML
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import UInt16, Bool
from ros_robot_controller.ros_robot_controller_sdk import Board, PacketReportKeyEvents
from ros_robot_controller_msgs.srv import GetBusServoState, GetPWMServoState
from ros_robot_controller_msgs.msg import (
    ButtonState, BuzzerState, MotorsState, BusServoState, LedState,
    SetBusServoState, ServosPosition, SetPWMServoState, Sbus, OLEDState,
    RGBStates, PWMServoState
)

class RosRobotController(Node):
    gravity = 9.80665

    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.board = Board()
        self.board.enable_reception()
        self.running = True

        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('init_finish', False)
        self.IMU_FRAME = self.get_parameter('imu_frame').value

        # 퍼블리셔 등록
        self.imu_pub = self.create_publisher(Imu, '~/imu_raw', 1)
        self.joy_pub = self.create_publisher(Joy, '~/joy', 1)
        self.sbus_pub = self.create_publisher(Sbus, '~/sbus', 1)
        self.button_pub = self.create_publisher(ButtonState, '~/button', 1)
        self.battery_pub = self.create_publisher(UInt16, '~/battery', 1)
        # 서브스크립션 등록
        self.create_subscription(LedState, '~/set_led', self.set_led_state, 5)
        self.create_subscription(BuzzerState, '~/set_buzzer', self.set_buzzer_state, 5)
        self.create_subscription(OLEDState, '~/set_oled', self.set_oled_state, 5)
        self.create_subscription(MotorsState, '~/set_motor', self.set_motor_state, 10)
        self.create_subscription(Bool, '~/enable_reception', self.enable_reception, 1)
        self.create_subscription(SetBusServoState, '~/bus_servo/set_state', self.set_bus_servo_state, 10)
        self.create_subscription(ServosPosition, '~/bus_servo/set_position', self.set_bus_servo_position, 10)
        self.create_subscription(SetPWMServoState, '~/pwm_servo/set_state', self.set_pwm_servo_state, 10)
        # 서비스 등록
        self.create_service(GetBusServoState, '~/bus_servo/get_state', self.get_bus_servo_state)
        self.create_service(GetPWMServoState, '~/pwm_servo/get_state', self.get_pwm_servo_state)
        self.create_subscription(RGBStates, '~/set_rgb', self.set_rgb_states, 10)

        # 加载并设置舵机偏移量从 YAML 文件
        self.load_servo_offsets()

        # 初始化电机速度
        self.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])

        self.clock = self.get_clock()
        threading.Thread(target=self.pub_callback, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def load_servo_offsets(self):
        """
        YAML 파일에서 서보 모터 오프셋값을 읽어와 적용
        """
        config_path = '/home/ubuntu/software/Servo_upper_computer/servo_config.yaml'
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)

            # 确保config是字典
            if not isinstance(config, dict):
                self.get_logger().error(f"YAML 설정 파일 형식 오류: {config_path}，应为字典格式。")
                return

            # ID1 ~ ID4에 대해 오프셋 적용
            for servo_id in range(1, 5):
                offset = config.get(servo_id, 0)  # 기본값 0
                try:
                    self.board.pwm_servo_set_offset(servo_id, offset)
                    self.get_logger().info(f"서보 {servo_id}의 오프셋을 {offset}으로 설정 완료")
                except Exception as e:
                    self.get_logger().error(f"서보 {servo_id} 오프셋 설정 중 오류: {e}")

        except FileNotFoundError:
            self.get_logger().error(f"설정 파일 없음: {config_path}")
        except yaml.YAMLError as e:
            self.get_logger().error(f"YAML 파일 파싱 오류: {e}")
        except Exception as e:
            self.get_logger().error(f"설정 파일 읽기 오류: {e}")

    def get_node_state(self, request, response):
        response.success = True
        return response

    # 센서 데이터 퍼블리시
    def pub_callback(self):
        while self.running:
            if getattr(self, 'enable_reception', False):
                self.pub_button_data(self.button_pub)   # 버튼상태
                self.pub_joy_data(self.joy_pub)     # 조이스틱
                self.pub_imu_data(self.imu_pub)     # IMU
                self.pub_sbus_data(self.sbus_pub)   # Sbus
                self.pub_battery_data(self.battery_pub)     # 베터리 전압
                time.sleep(0.02)
            else:
                time.sleep(0.02)
        rclpy.shutdown()

    def enable_reception(self, msg):
        self.get_logger().info('\033[1;32m%s\033[0m' % ('enable_reception ' + str(msg.data)))
        self.enable_reception = msg.data
        self.board.enable_reception(msg.data)

    # LED 제어
    def set_led_state(self, msg):
        self.board.set_led(msg.on_time, msg.off_time, msg.repeat, msg.id)

    # 부저 제어
    def set_buzzer_state(self, msg):
        self.board.set_buzzer(msg.freq, msg.on_time, msg.off_time, msg.repeat)
    
    # RGB LED
    def set_rgb_states(self, msg):
        pixels = []
        for state in msg.states:
            pixels.append((state.index, state.red, state.green, state.blue))
        self.board.set_rgb(pixels)

    # 모터 제어
    def set_motor_state(self, msg):
        data = []
        for i in msg.data:
            data.extend([[i.id, i.rps]])
        self.board.set_motor_speed(data)

    # OLED
    def set_oled_state(self, msg):
        self.board.set_oled_text(int(msg.index), msg.text)

    # PWM 서보
    def set_pwm_servo_state(self, msg):
        data = []
        for i in msg.state:
            if i.id and i.position:
                data.extend([[i.id[0], i.position[0]]])
            if i.id and i.offset:
                self.board.pwm_servo_set_offset(i.id[0], i.offset[0])

        if data != []:
            self.board.pwm_servo_set_position(msg.duration, data)

    def get_pwm_servo_state(self, msg):
        states = []
        for i in msg.cmd:
            data = PWMServoState()
            if i.get_position:
                state = self.board.pwm_servo_read_position(i.id)
                if state is not None:
                    data.position = state
            if i.get_offset:
                state = self.board.pwm_servo_read_offset(i.id)
                if state is not None:
                    data.offset = state
            states.append(data)
        return [True, states]

    # Bus 서보
    def set_bus_servo_position(self, msg):
        data = []
        for i in msg.position:
            data.extend([[i.id, i.position]])
        if data:
            self.board.bus_servo_set_position(msg.duration, data)

    # Bus 서보
    def set_bus_servo_state(self, msg):
        data = []
        servo_id = []
        for i in msg.state:
            if i.present_id:
                if i.present_id[0]:
                    if i.target_id:
                        if i.target_id[0]:
                            self.board.bus_servo_set_id(i.present_id[1], i.target_id[1])
                    if i.position:
                        if i.position[0]:
                            data.extend([[i.present_id[1], i.position[1]]])
                    if i.offset:
                        if i.offset[0]:
                            self.board.bus_servo_set_offset(i.present_id[1], i.offset[1])
                    if i.position_limit:
                        if i.position_limit[0]:
                            self.board.bus_servo_set_angle_limit(i.present_id[1], i.position_limit[1:])
                    if i.voltage_limit:
                        if i.voltage_limit[0]:
                            self.board.bus_servo_set_vin_limit(i.present_id[1], i.voltage_limit[1:])
                    if i.max_temperature_limit:
                        if i.max_temperature_limit[0]:
                            self.board.bus_servo_set_temp_limit(i.present_id[1], i.max_temperature_limit[1])
                    if i.enable_torque:
                        if i.enable_torque[0]:
                            self.board.bus_servo_enable_torque(i.present_id[1], i.enable_torque[1])
                    if i.save_offset:
                        if i.save_offset[0]:
                            self.board.bus_servo_save_offset(i.present_id[1])
                    if i.stop:
                        if i.stop[0]:
                            servo_id.append(i.present_id[1])
        if data != []:
            self.board.bus_servo_set_position(msg.duration, data)
        if servo_id != []:    
            self.board.bus_servo_stop(servo_id)

    def get_bus_servo_state(self, request, response):
        states = []
        for i in request.cmd:
            data = BusServoState()
            if i.get_id:
                state = self.board.bus_servo_read_id(i.id)
                if state is not None:
                    i.id = state[0]
                    data.present_id = state
            if i.get_position:
                state = self.board.bus_servo_read_position(i.id)
                if state is not None:
                    data.position = state
            if i.get_offset:
                state = self.board.bus_servo_read_offset(i.id)
                if state is not None:
                    data.offset = state
            if i.get_voltage:
                state = self.board.bus_servo_read_voltage(i.id)
                if state is not None:
                    data.voltage = state
            if i.get_temperature:
                state = self.board.bus_servo_read_temp(i.id)
                if state is not None:
                    data.temperature = state
            if i.get_position_limit:
                state = self.board.bus_servo_read_angle_limit(i.id)
                if state is not None:
                    data.position_limit = state
            if i.get_voltage_limit:
                state = self.board.bus_servo_read_vin_limit(i.id)
                if state is not None:
                    data.voltage_limit = state
            if i.get_max_temperature_limit:
                state = self.board.bus_servo_read_temp_limit(i.id)
                if state is not None:
                    data.max_temperature_limit = state
            if i.get_torque_state:
                state = self.board.bus_servo_read_torque(i.id)
                if state is not None:
                    data.enable_torque = state
            states.append(data)
        response.state = states
        response.success = True
        return response

    def pub_battery_data(self, pub):
        data = self.board.get_battery()
        if data is not None:
            msg = UInt16()
            msg.data = data
            pub.publish(msg)

    def pub_button_data(self, pub):
        data = self.board.get_button()
        if data is not None:
            key_id, key_event = data
            state_map = {
                PacketReportKeyEvents.KEY_EVENT_PRESSED: 1,
                PacketReportKeyEvents.KEY_EVENT_LONGPRESS: 2,
                PacketReportKeyEvents.KEY_EVENT_LONGPRESS_REPEAT: 3,
                PacketReportKeyEvents.KEY_EVENT_RELEASE_FROM_LP: 4,
                PacketReportKeyEvents.KEY_EVENT_RELEASE_FROM_SP: 0,
                PacketReportKeyEvents.KEY_EVENT_CLICK: 5,
                PacketReportKeyEvents.KEY_EVENT_DOUBLE_CLICK: 6,
                PacketReportKeyEvents.KEY_EVENT_TRIPLE_CLICK: 7,
            }
            state = state_map.get(key_event, -1)

            if state != -1:
                msg = ButtonState()
                msg.id = key_id
                msg.state = state
                pub.publish(msg)
            else:
                self.get_logger().error(f"Unhandled button event: {key_event}")

    def pub_joy_data(self, pub):
        data = self.board.get_gamepad()
        if data is not None:
            msg = Joy()
            msg.axes = data[0]
            msg.buttons = data[1]
            msg.header.stamp = self.clock.now().to_msg()
            pub.publish(msg)

    def pub_sbus_data(self, pub):
        data = self.board.get_sbus()
        if data is not None:
            msg = Sbus()
            msg.channel = data
            msg.header.stamp = self.clock.now().to_msg()
            pub.publish(msg)

    # IMU 데이터 처리
    def pub_imu_data(self, pub):
        data = self.board.get_imu()
        if data is not None:
            ax, ay, az, gx, gy, gz = data
            msg = Imu()
            msg.header.frame_id = self.IMU_FRAME
            msg.header.stamp = self.clock.now().to_msg()

            msg.orientation.w = 0.0
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0

            msg.linear_acceleration.x = ax * self.gravity
            msg.linear_acceleration.y = ay * self.gravity
            msg.linear_acceleration.z = az * self.gravity

            msg.angular_velocity.x = math.radians(gx)
            msg.angular_velocity.y = math.radians(gy)
            msg.angular_velocity.z = math.radians(gz)

            msg.orientation_covariance = [0.01, 0.0, 0.0,
                                          0.0, 0.01, 0.0,
                                          0.0, 0.0, 0.01]
            msg.angular_velocity_covariance = [0.01, 0.0, 0.0,
                                              0.0, 0.01, 0.0,
                                              0.0, 0.0, 0.01]
            msg.linear_acceleration_covariance = [0.0004, 0.0, 0.0,
                                                 0.0, 0.0004, 0.0,
                                                 0.0, 0.0, 0.004]
            pub.publish(msg)

def main():
    node = RosRobotController('ros_robot_controller')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 안전 종료: 모터 속도 0으로 설정
        node.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
        node.destroy_node()
        rclpy.shutdown()
        print('shutdown')
    finally:
        print('shutdown finish')

if __name__ == '__main__':
    main()
