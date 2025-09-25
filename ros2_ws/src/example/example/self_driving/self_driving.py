#!/usr/bin/env python3  # 실행 시 사용할 인터프리터 경로 지정
# encoding: utf-8  # 파일 인코딩을 UTF-8로 설정 (한글 주석 사용 가능)
# @data:2023/03/28  # 작성 날짜
# @author:aiden  # 작성자
# autonomous driving  # 이 파일은 자율주행 기능을 구현한 ROS2 노드입니다

import os  # 운영체제 관련 유틸리티 (환경변수 등)
import cv2  # OpenCV: 이미지 처리 라이브러리
import math  # 수학 함수 (삼각함수 등)
import time  # 시간 관련 함수 (sleep, time 등)
import queue  # 스레드 안전한 큐
import rclpy  # ROS2 Python 클라이언트 라이브러리
import threading  # 스레드 생성/관리
import numpy as np  # 수치 연산 라이브러리
import sdk.pid as pid  # 내부 SDK의 PID 제어기 모듈
import sdk.fps as fps  # 프레임 속도 계산기 모듈
from rclpy.node import Node  # ROS2 노드 베이스 클래스
import sdk.common as common  # 내부 공용 유틸리티
# from app.common import Heart  # 주석 처리된 Heart 모듈 (심박/헬스 체크용)
from cv_bridge import CvBridge  # ROS Image <-> OpenCV 변환 유틸
from sensor_msgs.msg import Image  # ROS sensor_msgs의 Image 메시지 타입
from geometry_msgs.msg import Twist  # 주행 명령용 Twist 메시지
from interfaces.msg import ObjectsInfo  # 커스텀 인터페이스: 객체 검출 결과 메시지 타입
from std_srvs.srv import SetBool, Trigger  # 표준 서비스 타입들
from sdk.common import colors, plot_one_box  # 이미지에 박스/색 지정 도우미 함수
from example.self_driving import lane_detect  # 같은 패키지의 lane_detect 모듈 불러오기
from rclpy.executors import MultiThreadedExecutor  # 멀티스레드 Executor
from rclpy.callback_groups import ReentrantCallbackGroup  # 재진입 콜백 그룹
from ros_robot_controller_msgs.msg import BuzzerState, SetPWMServoState, PWMServoState  # 로봇 컨트롤러 메시지들


class SelfDrivingNode(Node):
    def __init__(self, name):
        rclpy.init()  # rclpy 초기화 (프로세스 전체에 대해 한 번 호출)
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        # Node 초기화: 이름을 받아 파라미터 자동 선언 허용
        self.name = name  # 노드 이름 저장
        self.is_running = True  # 노드 루프 실행 플래그
        self.pid = pid.PID(0.4, 0.0, 0.05)  # PID 컨트롤러 초기화 (Kp, Ki, Kd)
        self.param_init()  # 내부 상태 변수 초기화 함수 호출

        self.fps = fps.FPS()  # FPS 측정기 초기화
        self.image_queue = queue.Queue(maxsize=2)  # 이미지 큐 (최대 2프레임 유지)
        self.classes = ['go', 'right', 'park', 'red', 'green', 'crosswalk']  # 디폴트 클래스 리스트 (초기값)
        self.display = True  # 결과 이미지 디스플레이 여부
        self.bridge = CvBridge()  # cv_bridge 인스턴스 생성
        self.lock = threading.RLock()  # 재진입 가능한 락 (스레드 동기화용)
        self.colors = common.Colors()  # 박스 색상 등을 제공하는 유틸
        # signal.signal(signal.SIGINT, self.shutdown)  # (주석) 시그널 핸들링 예시
        self.machine_type = os.environ.get('MACHINE_TYPE')  # 환경변수에서 머신 타입 조회
        self.lane_detect = lane_detect.LaneDetector("yellow")  # 차선 검출기 생성 (노란색 차선 대상)

        # 퍼블리셔 생성: 주행 명령을 보낼 토픽
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        # 서보 상태 설정 퍼블리셔 (사용처는 있음)
        self.servo_state_pub = self.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 1)
        # 결과 이미지를 퍼블리시할 퍼블리셔
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)

        # 서비스 서버 생성: enter/exit/set_running
        self.create_service(Trigger, '~/enter', self.enter_srv_callback) # enter the game
        self.create_service(Trigger, '~/exit', self.exit_srv_callback) # exit the game
        self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)
        # self.heart = Heart(self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(None))  # (주석) heartbeat 예시
        timer_cb_group = ReentrantCallbackGroup()  # 타이머용 재진입 콜백 그룹 생성
        self.client = self.create_client(Trigger, '/yolov5_ros2/init_finish')  # YOLO init 완료용 클라이언트
        self.client.wait_for_service()  # 해당 서비스가 준비될 때까지 블록
        # YOLO 시작/중지 서비스 클라이언트 생성 (콜백 그룹 지정)
        self.start_yolov5_client = self.create_client(Trigger, '/yolov5/start', callback_group=timer_cb_group)
        self.start_yolov5_client.wait_for_service()
        self.stop_yolov5_client = self.create_client(Trigger, '/yolov5/stop', callback_group=timer_cb_group)
        self.stop_yolov5_client.wait_for_service()

        # 타이머를 이용해 init_process를 한 번 실행하도록 예약 (0.0으로 즉시 실행)
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()  # 타이머 취소 (한 번만 실행하기 위함)

        self.mecanum_pub.publish(Twist())  # 안전을 위해 정지 토픽 퍼블리시
        if not self.get_parameter('only_line_follow').value:
            self.send_request(self.start_yolov5_client, Trigger.Request())  # YOLO 시작 서비스 호출
        time.sleep(1)  # YOLO 시작 안정화를 위해 잠시 대기
        
        if 1:#self.get_parameter('start').value:
            self.display = True  # 결과 디스플레이 활성화
            self.enter_srv_callback(Trigger.Request(), Trigger.Response())  # enter 서비스 콜백 직접 호출하여 구독 등 설정
            request = SetBool.Request()
            request.data = True
            self.set_running_srv_callback(request, SetBool.Response())  # 런 상태 설정

        #self.park_action()  # (주석) 바로 주차 동작 실행 예시
        threading.Thread(target=self.main, daemon=True).start()  # 메인 루프를 별도 데몬 스레드로 시작
        self.create_service(Trigger, '~/init_finish', self.get_node_state)  # 초기화 완료 확인용 서비스 생성
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')  # 시작 로그 출력

    def param_init(self):
        # 주행 및 상태 관련 변수 초기화
        self.start = False
        self.enter = False
        self.right = True

        self.have_turn_right = False
        self.detect_turn_right = False
        self.detect_far_lane = False
        self.park_x = -1  # 주차 표지판 x 픽셀 좌표 (초기값)

        self.start_turn_time_stamp = 0
        self.count_turn = 0
        self.start_turn = False  # 회전 시작 플래그

        self.count_right = 0
        self.count_right_miss = 0
        self.turn_right = False  # 우회전 표지판 플래그

        self.last_park_detect = False
        self.count_park = 0  # 주차 검출 카운터
        self.stop = False  # 정지 상태 플래그
        self.start_park = False  # 주차 시작 플래그

        self.count_crosswalk = 0
        self.crosswalk_distance = 0  # 횡단보도까지의 "거리" (이미지 좌표 기반)
        self.crosswalk_length = 0.1 + 0.3  # 횡단보도 길이 + 로봇 길이(초기값)

        self.start_slow_down = False  # 감속 상태 플래그
        self.normal_speed = 0.1  # 기본 주행 속도
        self.slow_down_speed = 0.1  # 감속 시 속도 (현재는 동일하게 설정되어 있음)

        self.traffic_signs_status = None  # 최근 신호등 감지 상태 저장
        self.red_loss_count = 0

        self.object_sub = None  # 서브스크립션 핸들 보관 (초기값 None)
        self.image_sub = None  # 이미지 서브스크립션 핸들
        self.objects_info = []  # 최근 감지된 객체 리스트

    def get_node_state(self, request, response):
        response.success = True  # 노드 상태 서비스는 항상 성공으로 응답
        return response

    def send_request(self, client, msg):
        future = client.call_async(msg)  # 비동기로 서비스 호출
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()  # 결과가 오면 반환 (현재 busy-wait 형태)

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "self driving enter")  # 로그 출력
        with self.lock:  # 스레드 안정성을 위해 락 사용
            self.start = False
            camera = 'depth_cam'#self.get_parameter('depth_camera_name').value  # 카메라 이름(주석처리된 기본값 있음)
            # 이미지 토픽과 객체검출 토픽을 구독 (구독 객체를 반환값으로 저장하지 않음 - 개선 여지)
            self.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image' , self.image_callback, 1)
            self.create_subscription(ObjectsInfo, '/yolov5_ros2/object_detect', self.get_object_callback, 1)
            self.mecanum_pub.publish(Twist())  # 정지 명령 퍼블리시
            self.enter = True
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "self driving exit")  # 로그 출력
        with self.lock:
            try:
                if self.image_sub is not None:
                    self.image_sub.unregister()  # 이미지 구독 해제 시도
                if self.object_sub is not None:
                    self.object_sub.unregister()  # 객체 구독 해제 시도
            except Exception as e:
                self.get_logger().info('\033[1;32m%s\033[0m' % str(e))  # 예외 내용 로깅
            self.mecanum_pub.publish(Twist())  # 정지 명령
        self.param_init()  # 상태 초기화
        response.success = True
        response.message = "exit"
        return response

    def set_running_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_running")  # 로그
        with self.lock:
            self.start = request.data  # start 플래그를 서비스 요청으로 설정
            if not self.start:
                self.mecanum_pub.publish(Twist())  # 중지 요청이면 정지 퍼블리시
        response.success = True
        response.message = "set_running"
        return response

    def shutdown(self, signum, frame):  # press 'ctrl+c' to close the program
        self.is_running = False  # 스레드 루프를 종료하도록 플래그 변경

    def image_callback(self, ros_image):  # callback target checking
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")  # ROS Image -> OpenCV 이미지로 변환 (rgb8 인코딩)
        rgb_image = np.array(cv_image, dtype=np.uint8)  # NumPy 배열로 변환
        if self.image_queue.full():
            # if the queue is full, remove the oldest image
            self.image_queue.get()  # 큐가 가득 차 있으면 가장 오래된 프레임을 버림
        # put the image into the queue
        self.image_queue.put(rgb_image)  # 새 프레임을 큐에 추가
    
    # parking processing
    def park_action(self):
        if self.machine_type == 'MentorPi_Mecanum': 
            twist = Twist()
            twist.linear.y = -0.2  # 메카넘 특성: y축으로 후진(측면 이동)
            self.mecanum_pub.publish(twist)
            time.sleep(0.38/0.2)  # 이동 시간 대기
        elif self.machine_type == 'MentorPi_Acker':
            twist = Twist()
            twist.linear.x = 0.15
            twist.angular.z = twist.linear.x*math.tan(-0.5061)/0.145  # Ackermann 스타일 계산
            self.mecanum_pub.publish(twist)
            time.sleep(3)

            twist = Twist()
            twist.linear.x = 0.15
            twist.angular.z = -twist.linear.x*math.tan(-0.5061)/0.145
            self.mecanum_pub.publish(twist)
            time.sleep(2)

            twist = Twist()
            twist.linear.x = -0.15
            twist.angular.z = twist.linear.x*math.tan(-0.5061)/0.145
            self.mecanum_pub.publish(twist)
            time.sleep(1.5)

        else:
            twist = Twist()
            twist.angular.z = -1  # 기본 회전 동작
            self.mecanum_pub.publish(twist)
            time.sleep(1.5)
            self.mecanum_pub.publish(Twist())
            twist = Twist()
            twist.linear.x = 0.2
            self.mecanum_pub.publish(twist)
            time.sleep(0.65/0.2)
            self.mecanum_pub.publish(Twist())
            twist = Twist()
            twist.angular.z = 1
            self.mecanum_pub.publish(twist)
            time.sleep(1.5)
        self.mecanum_pub.publish(Twist())  # 주차 동작 끝나면 정지 명령 퍼블리시

    def main(self):
        while self.is_running:
            time_start = time.time()  # 루프 시작 시간 기록 (주기 유지용)
            try:
                image = self.image_queue.get(block=True, timeout=1)  # 큐에서 이미지 획득, 타임아웃 1초
            except queue.Empty:
                if not self.is_running:
                    break
                else:
                    continue

            result_image = image.copy()  # 결과 시각화를 위한 복사본 생성
            if self.start:
                h, w = image.shape[:2]  # 이미지 높이, 너비

                # obtain the binary image of the lane
                binary_image = self.lane_detect.get_binary(image)  # 차선 이진화 마스크 생성

                twist = Twist()  # 주행 명령 초기화

                # if detecting the zebra crossing, start to slow down
                self.get_logger().info('\033[1;33m%s\033[0m' % self.crosswalk_distance)  # 디버그: 횡단보도 거리 로그
                if 70 < self.crosswalk_distance and not self.start_slow_down:  # crosswalk에서의 거리가 70 이상(너무 가깝지 않을때) 와 slow_down중이지 않을때
                    self.count_crosswalk += 1 # crosswalk 볼때 마다 count_crosswalk += 1
                    if self.count_crosswalk == 3:  # crosswalk을 3번 이상 감지했을때
                        self.count_crosswalk = 0 # count_crosswalk 리셋
                        self.start_slow_down = True  # slown down 하기
                        self.count_slow_down = time.time()  # fixing time for slowing down
                else:  # need to detect continuously, otherwise reset
                    self.count_crosswalk = 0

                # deceleration processing
                if self.start_slow_down: # start_slow_down = True이면 실행
                    if self.traffic_signs_status is not None: # traffic_signs_status가 비어있지 않으면
                        area = abs(self.traffic_signs_status.box[0] - self.traffic_signs_status.box[2]) * abs(self.traffic_signs_status.box[1] - self.traffic_signs_status.box[3]) # 모델이 인식하는 박스의 넓이 구하기
                        if self.traffic_signs_status.class_name == 'red' and area < 1000:  # 빨간불일때 & 신호등 박스 넓이가 1000 이하 일때 정지
                            self.mecanum_pub.publish(Twist())
                            self.stop = True
                        elif self.traffic_signs_status.class_name == 'green':  # 녹색불일때는 느리게 주행
                            self.mecanum_pub.publish(Twist())
                            twist.linear.x = self.slow_down_speed
                            self.stop = False
                    if not self.stop:  # 로봇이 정지하지 않은 다른 경우에는 속도를 늦추고 횡단보도를 통과하는 데 걸리는 시간을 계산합니다. 소요 시간은 횡단보도 길이 나누기 주행 속도
                        twist.linear.x = self.slow_down_speed
                        if time.time() - self.count_slow_down > self.crosswalk_length / twist.linear.x:
                            self.start_slow_down = False
                else:
                    twist.linear.x = self.normal_speed  # go straight with normal speed

                if 0 < self.park_x and 135 < self.crosswalk_distance: # crosswalk에서의 거리가 135 이상(너무 가깝지 않을때) 와 parking 표지판을 봤을때
                    twist.linear.x = self.slow_down_speed
                    if not self.start_park and 180 < self.crosswalk_distance:  # crosswalk에서의 거리가 180 이상(너무 가깝지 않을때) 와 주차를 아직 시작 안했을때
                        self.count_park += 1  # parking 표지판 볼때 마다 count_park += 1
                        if self.count_park >= 15:  # parking표지판을 15번 이상 감지했을때 
                            self.mecanum_pub.publish(Twist())  
                            self.start_park = True # 주차 시작
                            self.stop = True # 정지
                            threading.Thread(target=self.park_action).start()
                    else:
                        self.count_park = 0  

                # line following processing
                result_image, lane_angle, lane_x = self.lane_detect(binary_image, image.copy())  # the coordinate of the line while the robot is in the middle of the lane
                if lane_x >= 0 and not self.stop:  
                    if lane_x > 150:  
                        self.count_turn += 1
                        if self.count_turn > 5 and not self.start_turn:
                            self.start_turn = True
                            self.count_turn = 0
                            self.start_turn_time_stamp = time.time()
                        if self.machine_type != 'MentorPi_Acker':
                            twist.angular.z = -0.45  # turning speed
                        else:
                            twist.angular.z = twist.linear.x * math.tan(-0.5061) / 0.145
                    else:  # use PID algorithm to correct turns on a straight road
                        self.count_turn = 0
                        if time.time() - self.start_turn_time_stamp > 2 and self.start_turn:
                            self.start_turn = False
                        if not self.start_turn:
                            self.pid.SetPoint = 130  # the coordinate of the line while the robot is in the middle of the lane
                            self.pid.update(lane_x)
                            if self.machine_type != 'MentorPi_Acker':
                                twist.angular.z = common.set_range(self.pid.output, -0.1, 0.1)
                            else:
                                twist.angular.z = twist.linear.x * math.tan(common.set_range(self.pid.output, -0.1, 0.1)) / 0.145
                        else:
                            if self.machine_type == 'MentorPi_Acker':
                                twist.angular.z = 0.15 * math.tan(-0.5061) / 0.145
                    self.mecanum_pub.publish(twist)  
                else:
                    self.pid.clear()

                # 지금 프레임에서 탐지된 모든 객체들의 정보 리스트
                if self.objects_info: 
                    for i in self.objects_info:
                        box = i.box
                        class_name = i.class_name
                        cls_conf = i.score
                        cls_id = self.classes.index(class_name)
                        color = colors(cls_id, True)
                        plot_one_box(
                            box,
                            result_image,
                            color=color,
                            label="{}:{:.2f}".format(class_name, cls_conf),
                        )

            else:
                time.sleep(0.01)

            
            bgr_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            if self.display:
                self.fps.update()
                bgr_image = self.fps.show_fps(bgr_image)

            
            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(bgr_image, "bgr8"))

           
            time_d = 0.03 - (time.time() - time_start)
            if time_d > 0:
                time.sleep(time_d)
        self.mecanum_pub.publish(Twist())
        rclpy.shutdown()

    # Obtain the target detection result
    def get_object_callback(self, msg):
        # self.objects_info에 메시지에서 추출한 객체 목록을 저장합니다.
        self.objects_info = msg.objects
        
        if self.objects_info == []:  # 감지된 객체가 없는 경우 (리스트가 비어있는 경우)
            self.traffic_signs_status = None # 교통 신호등 상태를 '없음'으로 재설정합니다.
            self.crosswalk_distance = 0      # 횡단보도까지의 거리를 0으로 재설정합니다.
        else: # 감지된 객체가 있는 경우
            min_distance = 0 # 가장 가까운(가장 아래쪽에 있는) 횡단보도의 center 좌표를 0으로 설정
            for i in self.objects_info:
                class_name = i.class_name # 객체의 클래스 이름을 가져옵니다.
                center = (int((i.box[0] + i.box[2])/2), int((i.box[1] + i.box[3])/2)) # 객체의 경계 상자(box)를 사용하여 중심 픽셀 좌표를 계산합니다.
                if class_name == 'crosswalk':  
                    # 중심 y좌표가 현재 min_distance보다 큰지 확인합니다.
                    # 이미지 좌표계에서 y값이 클수록(숫자가 높을수록) 보통 객체가 아래쪽, 즉 더 가깝다는 의미입니다.
                    if center[1] > min_distance:  # 횡단보도의 가장 가까운 중심 픽셀 좌표를 얻습니다.
                        min_distance = center[1]
                elif class_name == 'right':
                    self.count_right += 1    # 감지 횟수를 증가시킵니다.
                    self.count_right_miss = 0 # 미인식 카운트를 초기화합니다.
                    if self.count_right >= 5:  # 5회 이상 연속으로 감지되면 우회전 상태를 True로 설정합니다. (노이즈 방지)
                        self.turn_right = True # 우회전 표지판을 'True'로 간주합니다.
                        self.count_right = 0   # 카운터를 다시 0으로 초기화합니다.
                elif class_name == 'park':  # 주차 표지판의 중심 x좌표를 얻습니다.
                    self.park_x = center[0]
                elif class_name == 'red' or class_name == 'green':  # 교통 신호등의 상태 정보를 얻습니다.
                    self.traffic_signs_status = i
            self.get_logger().info('\033[1;32m%s\033[0m' % class_name)
            # 가장 가까운 횡단보도의 y축 픽셀 좌표를 인스턴스 변수에 저장합니다.
            self.crosswalk_distance = min_distance

def main():
    # 'SelfDrivingNode' 클래스의 인스턴스를 'self_driving'이라는 이름으로 생성합니다. (ROS 2 노드 생성)
    node = SelfDrivingNode('self_driving')
    # 다중 스레드 실행자(Executor)를 생성하여 여러 콜백을 동시에 처리할 수 있게 합니다.
    executor = MultiThreadedExecutor()
    # 노드를 실행자에 추가합니다.
    executor.add_node(node)
    # 노드의 콜백이 실행되도록 스핀(반복 실행)합니다. (메인 루프)
    executor.spin()
    # 프로그램이 종료될 때 노드를 파괴(정리)합니다.
    node.destroy_node()

if __name__ == "__main__":
    # 스크립트가 직접 실행될 때 'main' 함수를 호출합니다.
    main()