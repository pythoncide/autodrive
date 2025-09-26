#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/03/28
# @author:aiden
# autonomous driving
import os
import cv2
import math
import time
import queue
import rclpy
import threading
import numpy as np
import sdk.pid as pid
import sdk.fps as fps
from rclpy.node import Node
import sdk.common as common
# from app.common import Heart
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from interfaces.msg import ObjectsInfo
from std_srvs.srv import SetBool, Trigger
from sdk.common import colors, plot_one_box
from example.self_driving import lane_detect
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from ros_robot_controller_msgs.msg import BuzzerState, SetPWMServoState, PWMServoState

# 클래스 선언 & 생성자
class SelfDrivingNode(Node): # 드라이빙 노드들
    def __init__(self, name): # 초기 설정
        rclpy.init() # ROS 통신 초기화(보통 main에서 하지만 여기서도 가능)

        # 미선언 파라미터 허용 & launch/CLI로 받은 파라미터 자동 선언
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name # 이름 인자 저장
        self.is_running = True # 메인 루프 런타임 플래그
        self.pid = pid.PID(0.4, 0.0, 0.05) # 조향(P 값 0.4, D 0.05) PID 컨트롤러
        self.param_init() # 런타임 상태 변수 초기화

        self.fps = fps.FPS()  # fps 표시
        self.image_queue = queue.Queue(maxsize=2) # 최신 프레임 2장만 보관
        self.classes = ['go', 'right', 'park', 'red', 'green', 'crosswalk'] # 클래스 이름 맵핑
        self.display = True # 결과 화면(이미지)에 FPS나 디텍션 박스 등을 표시할지 말지
        self.bridge = CvBridge() # ROS ↔ OpenCV 변환기
        self.lock = threading.RLock() # 재진입 가능 락(서비스 콜백 동시성용)
        self.colors = common.Colors() # 색상 팔레트(미사용도 일부 있음)
        # signal.signal(signal.SIGINT, self.shutdown)
        self.machine_type = os.environ.get('MACHINE_TYPE') # 구동 플랫폼 종류
        self.lane_detect = lane_detect.LaneDetector("yellow") # 노란 차선 기준

        # 퍼블리셔들 (메시지 타입, 토픽 이름, 큐 사이즈(몇개 쌓아둘것인지)), 퍼블리시를 하면 로보카가 움직인다.
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1) # Twist(직선속도, 각속도)

        # SetPWMServoState(서보번호, 원하는 각도, 동작시간)
        self.servo_state_pub = self.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 1)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1) # Image(이미지 전송)

        # 서비스 서버: 시작/종료/실행 토글
        self.create_service(Trigger, '~/enter', self.enter_srv_callback) # enter the game # 서비스 요청이 들어오면 시작 콜백 함수 실행
        self.create_service(Trigger, '~/exit', self.exit_srv_callback) # exit the game
        self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)
        # self.heart = Heart(self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(None))
        timer_cb_group = ReentrantCallbackGroup() # 타이머&클라이언트 재진입 허용
        self.client = self.create_client(Trigger, '/yolov5_ros2/init_finish') # 서비스 서버 클라이언트 만들기 (이름 : /yolov_ros2/init_finish)
        self.client.wait_for_service() # 서비스가 살아날 때가지 기다림
        self.start_yolov5_client = self.create_client(Trigger, '/yolov5/start', callback_group=timer_cb_group) # yolov5 서버 생성, 
        self.start_yolov5_client.wait_for_service() # 생성 시 까지 기다리기
        self.stop_yolov5_client = self.create_client(Trigger, '/yolov5/stop', callback_group=timer_cb_group) # yolov5 서버 종료
        self.stop_yolov5_client.wait_for_service() # 종료 시 까지 기다리기

        # 0초 주기 타이머: spin이 시작되면 즉시 init_process 한 번 실행
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    # 초기화 타이머 콜백
    def init_process(self): # 초기 프로세스
        self.timer.cancel() # 타이머 중지 (한번만 실행될 수 있도록)

        self.mecanum_pub.publish(Twist()) # 일단 정지 (노드 준비)
        if not self.get_parameter('only_line_follow').value: # only_line_follow 파라미터 없다면
            self.send_request(self.start_yolov5_client, Trigger.Request()) # YOLO 시작 트리거
        time.sleep(1) # 잠시 대기 (노드 대기시간?)
        
        if 1:#self.get_parameter('start').value: <- 이거 대체용, 이러면 start는 항상 True
            self.display = True # 결과 화면(이미지)에 FPS나 디텍션 박스 등을 표시
            
            # Trigger 타입 요청, 응답객체를 만들어 enter_srv_callback 함수 강제 실행
            self.enter_srv_callback(Trigger.Request(), Trigger.Response())
            request = SetBool.Request() # True, False 보내는 SetBool 객체 생성
            request.data = True # 객체 data에 True 입력

            # request와 응답객체를 직접 넘겨서 set_running_srv_callback 함수 강제 실행
            self.set_running_srv_callback(request, SetBool.Response())
            # 외부에서 호출이 들어와야 시작하는걸 내부에서 강제 시작한다.

        #self.park_action() 
        threading.Thread(target=self.main, daemon=True).start() # 프레임 처리 스레드 시작

        # /self_driving/init_finish 서비스 호출 시 self.get_node_state 콜백 실행, 준비완료 신호 회신
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start') # 초록색으로 로그 송출

    # 런타임 상태 변수 초기화
    def param_init(self): # 초기 파라미터
        self.start = False # 주행(start) 하고 있는지
        self.enter = False # 노드 시작준비(enter) 하고있는지
        self.right = True # 우회전 허용 가능한지

        self.have_turn_right = False # 우회전 이미 했는지
        self.detect_turn_right = False # 우회전 표지판 감지 했는지
        self.detect_far_lane = False # 먼거리 차선 감지 햇는지

        # Parking 표지판의 x좌표는 몇인지
        self.park_x = -1  # obtain the x-pixel coordinate of a parking sign

        self.start_turn_time_stamp = 0 # 회전 시작시간 기록
        self.count_turn = 0 # 회전 조건 카운트
        self.start_turn = False  # start to turn # 실제 회전 중인지

        self.count_right = 0 # 우회전 표지판 연속 감지 횟수
        self.count_right_miss = 0 # 우회전 표지판 놓치는 횟수
        self.turn_right = False  # right turning sign # 실제로 우회전 중인지

        self.last_park_detect = False # 직전 프레임에 주차 표지판을 감지했는지 -> 지속적으로 주차 표지 감지 사용
        self.count_park = 0  # 주차 표지 감지 누적 횟수 카운트
        self.stop = False  # stopping sign # 로보카가 멈춰야 되는지
        self.start_park = False  # start parking sign # 실제로 주차를 시작 했는지

        self.count_crosswalk = 0 # 횡단보도 인식 카운터

        # 횡단보도까지 거리 (픽셀 단위)
        self.crosswalk_distance = 0  # distance to the zebra crossing

        # 횡단 보도 구간 길이 추정 (횡단보도 폭 = 0.1m) + (로봇 자체 길이 = 0.3m)
        self.crosswalk_length = 0.1 + 0.3  # the length of zebra crossing and the robot

        self.start_slow_down = False  # slowing down sign # 감속을 시작해야 하는지
        self.normal_speed = 0.1  # normal driving speed # 기본 직진 속도
        self.slow_down_speed = 0.1  # slowing down speed # 감속 시 속도

        # 신호등 객체 정보 저장
        self.traffic_signs_status = None  # record the state of the traffic lights
        self.red_loss_count = 0 # 빨간불 신호 감지 후 놓치는 횟수 -> 잠깐 사라져도 정지해제 방지용

        self.object_sub = None # 객체 인식 구독 저장
        self.image_sub = None # 카메라 이미지 구독 저장
        self.objects_info = [] # 객체 검출 결과 리스트

    # 간단 서비스/유틸함수들
    def get_node_state(self, request, response): # ROS2 서비스 노드 상태 확인용 # 타이머 콜백 호출
        response.success = True # 응답 성공
        return response # 항상 True 응답

    # 클라이언트에 요청보내고 응답 기다리는 함수
    def send_request(self, client, msg):
        future = client.call_async(msg) # async(비동기)방식으로 요청 보냄, future는 응답 대기 객체
        while rclpy.ok(): # ROS2 정상 실행 시
            if future.done() and future.result(): # 응답이 도착하거나 응답 데이터를 가져오면
                return future.result() # 응답 데이터 리턴
    
    # 서비스 호출 시 ROS2가 실행해주는 콜백 함수, enter 서비스 콜백 함수
    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "self driving enter") # 초록색 로그 출력
        with self.lock: # 다른 콜백이 상태를 못 바꾸게 진입 방지
            self.start = False # 주행 상태 초기화 -> 주행 안함
            camera = 'depth_cam'#self.get_parameter('depth_camera_name').value

            # 카메라 이미지 구독 시작 : 카메라 퍼블리셔가 Image 타입을 보내면 image_callback 실시
            self.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image' , self.image_callback, 1)
            # YOLOv5 객체 검출 구독 시작 : 객체 검출 퍼블리셔가 ObjectInfo 타입을 보내면 self.get_object_callback 실시
            self.create_subscription(ObjectsInfo, '/yolov5_ros2/object_detect', self.get_object_callback, 1)
            # create_subscription 구독 객체 반환 시 변수 저장 없음...파이썬 GC에서 삭제 가능
            # 저장이 필요하다면 위 두줄은 변수 지정 필요
            
            self.mecanum_pub.publish(Twist()) # 로보카 속도 0, 정지
            self.enter = True # ***노드 시작 준비 트리거***
        response.success = True # 서비스 응답 상태 success: true
        response.message = "enter" # 서비스 응답 메세지 message: "enter"
        return response # 응답 상태 반환
    
    # exit 서비스 콜백 함수
    def exit_srv_callback(self, request, response): 
        self.get_logger().info('\033[1;32m%s\033[0m' % "self driving exit") # 초록색 로그 출력
        with self.lock: # 다른 콜백이 상태를 못 바꾸게 진입 방지
            try:
                if self.image_sub is not None: # 이미지 구독 저장이 False면
                    self.image_sub.unregister() # 이미지 구독 해제
                    # 단 .unregiste()는 최신rclpy에서 대신
                    # self.destroy_subscription(self.image_sub) 이걸 쓴다고 한다.

                if self.object_sub is not None: # 객체 탐지 구독 저장이 False면
                    self.object_sub.unregister() # 객체 탐지 구독 해제
            except Exception as e: # 구독 해제 중 에러가 난다면
                self.get_logger().info('\033[1;32m%s\033[0m' % str(e)) # 로그 출력
            self.mecanum_pub.publish(Twist()) # 로보카 속도 0 -> 정지
        self.param_init() #런타임 변수 초기화
        response.success = True # 응답 값
        response.message = "exit" # 응답 값
        return response # 응답 상태 반환

    # running 서비스 콜백 함수
    def set_running_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_running") # 로그 출력
        with self.lock: # 스레드 충돌 방지 (다른 콜백이 값 못 바꾸게 진입 방지)
            self.start = request.data # 리퀘스트 데이터(True or False)에 스타트 플래그 저장
            if not self.start: # False, 주행 정지라면
                self.mecanum_pub.publish(Twist()) # 로보카 속도 0 -> 정지
        response.success = True # 서비스 응답 값
        response.message = "set_running" # 서비스 응답 값
        return response # 서비스 응답값 출력

    # 종료키 (Ctrl + C) 콜백 함수
    def shutdown(self, signum, frame):  # press 'ctrl+c' to close the program
        self.is_running = False # 메인 루프 멈추는 플래그
        # -> main() 루프 빠져나오며 rclpy.shutdown() 호출

    # 이미지 콜백 함수
    def image_callback(self, ros_image):  # callback target checking

        # OpenCV 이미지 변환 8비트 RGB 컬러, numpy.ndarray 배열
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")

        # OpenCV 이미지 numpy uint8로 확실히 보정
        rgb_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full(): # 이미지가 꽉 차있다면
            # if the queue is full, remove the oldest image
            self.image_queue.get() # 오래된 이미지부터 꺼내버린다.
        # put the image into the queue
        self.image_queue.put(rgb_image) # 그개 아니라면 이미지를 추가한다.
    
    # parking processing
    # 주차 동작 함수
    def park_action(self):
        # 메카넘 휠 일 경우
        if self.machine_type == 'MentorPi_Mecanum': #환경
            twist = Twist() # ROS2 객체 twist 생성 (선속도, 각속도)
            twist.linear.y = -0.2 # y 방향 속도 -0.2 m/s
            self.mecanum_pub.publish(twist) # 메카넘 휠 동작에 퍼블리시 -> 모터 동작
            time.sleep(0.38/0.2) # 거리 / 속도 = 시간 -> 38cm를 0.2m/s로 1.9초 동안 주행
        elif self.machine_type == 'MentorPi_Acker':
            twist = Twist()
            twist.linear.x = 0.15
            twist.angular.z = twist.linear.x*math.tan(-0.5061)/0.145
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

        else: # 둘다 아니라면
            twist = Twist()
            twist.angular.z = -1 # 반시계 방향 회전
            self.mecanum_pub.publish(twist)
            time.sleep(1.5) # 1.5초 동작
            self.mecanum_pub.publish(Twist()) # 정지
            twist = Twist()
            twist.linear.x = 0.2 # x축 0.2m/s 속도
            self.mecanum_pub.publish(twist)
            time.sleep(0.65/0.2) # 0.65m를 0.2m/s 속도로 -> 3.25초
            self.mecanum_pub.publish(Twist()) # 정지
            twist = Twist()
            twist.angular.z = 1 # 시계 방향 회전
            self.mecanum_pub.publish(twist)
            time.sleep(1.5) # 1.5초 동작
        self.mecanum_pub.publish(Twist()) # 정지

    # 메인 주행 함수
    def main(self):
        while self.is_running: # is_running 트리거가 계속 True 일 때
            time_start = time.time() # 루프 시작 시간 기록
            try:
                # 카메라 이미지 들어올 때까지 1초 대기, 최신 이미지 추출, 없으면 queue.Empty
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.is_running: # 루프 해제
                    break
                else:
                    continue
            
            result_image = image.copy() # 받은 이미지를 복사해서 result_image에 저장
            
            # 주행시작 플래그가 True 일 때
            if self.start:
                h, w = image.shape[:2] # 이미지 높이, 너비 추출

                # obtain the binary image of the lane
                # 노란차선만 필터링하여 이진화 이미지 반환
                binary_image = self.lane_detect.get_binary(image)

                twist = Twist() # 속도 메세지 생성

                # if detecting the zebra crossing, start to slow down
                self.get_logger().info('\033[1;33m%s\033[0m' % self.crosswalk_distance) # 로그 출력
                # 조건#1 횡단보도거리가 70픽셀보다 클 때(가까워짐)
                # 조건#2 감속모드 시작 안됬음
                if 70 < self.crosswalk_distance and not self.start_slow_down:  # The robot starts to slow down only when it is close enough to the zebra crossing
                    self.count_crosswalk += 1 # 프레임 조건에 +1 추가

                    # 프레임에 3번 이상 찍혔을 때
                    if self.count_crosswalk == 3:  # judge multiple times to prevent false detection
                        self.count_crosswalk = 0 # 프레임 조건 초기화
                        
                        # 감속 시작
                        self.start_slow_down = True  # sign for slowing down
                        # 감속 시작 시간 기록
                        self.count_slow_down = time.time()  # fixing time for slowing down
                else:  # need to detect continuously, otherwise reset
                    # 조건이 끊기면 프레임 조건 초기화
                    self.count_crosswalk = 0

                # deceleration processing
                # 감속 모드 실행 시
                if self.start_slow_down:

                    # 신호등 정보가 있다면 (정보는 get_object_callback에서 업데이트)
                    if self.traffic_signs_status is not None:

                        # 신호등 바운딩 박스 계산
                        # self.traffic_signs_status.box 는 신호등 객체 바운딩 박스 좌표 배열
                        # [x_min, y_min, x_max, y_max]
                        # ...[0] - ...[2]는 x크기 (바운딩 박스 너비)
                        # ...[1] - ...[3]은 y크기 (바운딩 박스 높이)
                        # 둘의 곱으로 넓이 구할 수 있다.
                        # abs는 절대값
                        area = abs(self.traffic_signs_status.box[0] - self.traffic_signs_status.box[2]) * abs(self.traffic_signs_status.box[1] - self.traffic_signs_status.box[3])
                        
                        # 조건#1 클래스네임이 red
                        # 조건#2 바운딩박스 넓이가 1000 미만
                        if self.traffic_signs_status.class_name == 'red' and area < 1000:  # If the robot detects a red traffic light, it will stop
                            self.mecanum_pub.publish(Twist()) # 로보카 정지
                            self.stop = True # 정지 상태 갱신
                        
                        # 클래스네임이 green이라면
                        elif self.traffic_signs_status.class_name == 'green':  # If the traffic light is green, the robot will slow down and pass through
                            # x축 속도가 slow_down_speed로 진행
                            twist.linear.x = self.slow_down_speed
                            # 정지 상태 아님 갱신
                            self.stop = False
                    
                    # 초록불을 받아 정지 상태가 아니라면
                    if not self.stop:  # In other cases where the robot is not stopped, slow down the speed and calculate the time needed to pass through the crosswalk. The time needed is equal to the length of the crosswalk divided by the driving speed
                        # x축 속도가 slow_down_speed로 진행
                        twist.linear.x = self.slow_down_speed
                        
                        #(현재 시각 - 감속 시작 시각) > (횡단보도 길이 / 속도) 라면 -> 횡단보도 구간을 다 지나가면
                        if time.time() - self.count_slow_down > self.crosswalk_length / twist.linear.x:
                            
                            # 감속 상태 아님 갱신
                            self.start_slow_down = False
                
                # 감속 모드가 아니라면
                else:
                    
                    # 원래 속도 출발
                    twist.linear.x = self.normal_speed  # go straight with normal speed

                # If the robot detects a stop sign and a crosswalk, it will slow down to ensure stable recognition
                # 조건#1 주차 표지판 감지 (픽셀 x 좌표가 0보다 클 때)
                # 조건#2 횡단보도 감지 (픽셀거리가 135보다 클 때, 어느 정도 거리가 된다면)
                if 0 < self.park_x and 135 < self.crosswalk_distance:
                    
                    # x 속도 감속모드 들어가기
                    twist.linear.x = self.slow_down_speed

                    # 조건#1 주차 시작 상태 아님
                    # 조건#2 횡단보도 거리가 180 이상일 때 (적당한 거리)
                    if not self.start_park and 180 < self.crosswalk_distance:  # When the robot is close enough to the crosswalk, it will start parking
                        
                        # 프레임 카운트 시작
                        self.count_park += 1  
                        
                        # 프레임 카운트가 15 이상일 때
                        if self.count_park >= 15:  
                            self.mecanum_pub.publish(Twist()) # 로보카 정지 신호
                            self.start_park = True # 주차 시작 갱신
                            self.stop = True # 주행 중단 갱신

                            # 새로운 스레드에서 self.park_action 시작
                            threading.Thread(target=self.park_action).start()

                    # 조건이 끊기면 카운트 초기화
                    else:
                        self.count_park = 0  

                # line following processing
                # 시각화 이미지, 차선 각도, 차선 중신 x좌표 획득
                result_image, lane_angle, lane_x = self.lane_detect(binary_image, image.copy())  # the coordinate of the line while the robot is in the middle of the lane
                
                # 조건#1 차선 검출 성공 (lane_x >=0)
                # 조건#2 차량 멈춤 상태가 아니라면
                if lane_x >= 0 and not self.stop:  
                    if lane_x > 150:  # x좌표가 150 이상이라면 (우측으로 치우침)
                        self.count_turn += 1 # 회전 카운트 시작

                        # 조건#1 카운트가 5 초과
                        # 조건#2 회전 상태가 아닐때
                        if self.count_turn > 5 and not self.start_turn:
                            self.start_turn = True # 회전 상태 갱신
                            self.count_turn = 0 # 카운트 초기화

                            # 회전 시작 시간 기록
                            self.start_turn_time_stamp = time.time()

                        # 메카넘 휠 일때 각속도 -0.45rad/s
                        if self.machine_type != 'MentorPi_Acker':
                            twist.angular.z = -0.45  # turning speed
                        else:                            
                            twist.angular.z = twist.linear.x * math.tan(-0.5061) / 0.145
                    
                    # x좌표가 150 미만일때 (직선 구간)
                    else:  # use PID algorithm to correct turns on a straight road
                        self.count_turn = 0 # 회전 상태 갱신

                        # 조건#1 (현재시간) - 회전 시작시간 이 2초보다 크면 (회전 2초 경과)
                        # 조건#2 회전상태라면
                        if time.time() - self.start_turn_time_stamp > 2 and self.start_turn:
                            self.start_turn = False # 회전 상태 정지 갱신

                        # 회전 상태 정지라면
                        if not self.start_turn:

                            # 목표 차선 x좌표 목표 차선 130으로 pid 오차 계산
                            self.pid.SetPoint = 130  # the coordinate of the line while the robot is in the middle of the lane
                            
                            # Setpoint - lane_x 값으로 pid 오차 계산 실시
                            # self.pid.output 값 저장
                            self.pid.update(lane_x)
                            if self.machine_type != 'MentorPi_Acker': # 아커만 휠이 아니라면
                                
                                # z축 회전 속도 = pid 결과값 -0.1~0.1 사이
                                twist.angular.z = common.set_range(self.pid.output, -0.1, 0.1)
                            else:
                                twist.angular.z = twist.linear.x * math.tan(common.set_range(self.pid.output, -0.1, 0.1)) / 0.145
                        
                        # 회전 상태 정지가 아니라면 (회전 중)
                        else:
                            # 아커만 휠이라면 (메카넘휠으로 해당 없음)
                            if self.machine_type == 'MentorPi_Acker':
                                twist.angular.z = 0.15 * math.tan(-0.5061) / 0.145

                    # 회전 속도 전송
                    self.mecanum_pub.publish(twist)  
                
                # 차선을 못찾거나 정지상태면
                else:
                    self.pid.clear() # PID 내부 리셋

                # 오브젝트 인포가 있다면 (객체 검출결과 리스트에 값이 있다면)
                if self.objects_info:
                    for i in self.objects_info: # 검출결과 리스트를 순회
                        box = i.box # 박스 키의 밸류 [x1, y1, x2, y2]
                        class_name = i.class_name # 클래스 키 밸류 ["red", "green"...]
                        cls_conf = i.score # 검출 신뢰도 키 밸류 0.92
                        cls_id = self.classes.index(class_name) # 클래스 인덱스 사용
                        color = colors(cls_id, True) # 클래스 인덱스에 대응하는 색상 받기
                        plot_one_box( # 박스 그리기
                            box, # 박스 좌표
                            result_image, # 대상 이미지
                            color=color, # 박스 선 색깔
                            label="{}:{:.2f}".format(class_name, cls_conf), # 라벨 : 클래스네임, 신뢰도
                        )

            # 주행 시작 상태가 아니라면
            else:
                time.sleep(0.01) # CPU 대기

            # result image를 RGB에서 BGR로 변환
            bgr_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            if self.display: # 디버그 오버레이 상태 라면
                self.fps.update() # fps 표시
                bgr_image = self.fps.show_fps(bgr_image) # 현재 FPS 값을 영상 위에 텍스트로 오버레이.

            # OpenCV 이미지를 ROS로 변환하여 전송
            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(bgr_image, "bgr8"))
            time_d = 0.03 - (time.time() - time_start) # 루프 한바퀴의 목표주기 (0.03 ~ 33Hz)
            if time_d > 0: # 양수라면 (목표주기를 못채웠을 때, 여유)
                time.sleep(time_d) # 여유만큼 대기, 고정 주기 충족
        self.mecanum_pub.publish(Twist()) # 로보카 정지
        rclpy.shutdown() # ROS2 노드 종료


    # Obtain the target detection result
    # 객체 탐지 콜백 함수
    def get_object_callback(self, msg):
        self.objects_info = msg.objects # YOLO가 검출한 객체 리스트를 self.objects_info에 저장
        
        # 검출 결과가 없다면
        if self.objects_info == []:  # If it is not recognized, reset the variable
            # 신호등 정지 상태 초기화
            self.traffic_signs_status = None

            # 횡단보도 거리 상태 초기화
            self.crosswalk_distance = 0

        # 검출 결과가 있다면
        else:
            min_distance = 0
            for i in self.objects_info: # self.objects_info를 하나씩 순회
                class_name = i.class_name # 클래스 네임 키 밸류 추출

                # 바운딩 박스 중심 좌표 (x_cent, y_cent)
                center = (int((i.box[0] + i.box[2])/2), int((i.box[1] + i.box[3])/2))
                
                # class_name이 crosswalk라면 / 객체가 횡단보도라면
                if class_name == 'crosswalk':  
                    
                    # 바운딩 중심 y좌표가 min_distance보다 크다면
                    if center[1] > min_distance:  # Obtain recent y-axis pixel coordinate of the crosswalk
                        min_distance = center[1] # min 값 갱신
                
                # 객체가 우회전이라면        
                elif class_name == 'right':  # obtain the right turning sign 
                    self.count_right += 1 # 우회전 프레임 카운트 시작
                    self.count_right_miss = 0 # 우회전 놓침 초기화

                    # 카운트가 5 이상일때
                    if self.count_right >= 5:  # If it is detected multiple times, take the right turning sign to true
                        self.turn_right = True # 우회전 시작 갱신
                        self.count_right = 0 # 카운트 초기화
                
                # 객체가 주차라면
                elif class_name == 'park':  # obtain the center coordinate of the parking sign
                    
                    # 바운딩 중심 좌표 x좌표를 Parking 표지판의 x좌표로 갱신
                    self.park_x = center[0]
                
                # 객체가 빨간색 또는 초록색이면
                elif class_name == 'red' or class_name == 'green':  # obtain the status of the traffic light
                    # 현재 i 값 저장
                    self.traffic_signs_status = i
               

            self.get_logger().info('\033[1;32m%s\033[0m' % class_name) # 로그 출력
            self.crosswalk_distance = min_distance # 로봇에 가장 가까운 횡단보도 거리 지정

# ROS 노드 메인 엔트리 포인트
def main(): # 메인 함수
    node = SelfDrivingNode('self_driving') # 처음 클래스 생성, self_driving으로 노드 초기화, ROS2에 노드 등록
    executor = MultiThreadedExecutor() # 멀티 스레드 실행
    executor.add_node(node) # 멀티스레드에 self_driving 노드 등록
    executor.spin() # 멀티스레드 실행, 루프를 돌며 프로그램이 계속 살아잇음
    node.destroy_node() # spin이 끝나면 노드 깨끗이 종료
 
if __name__ == "__main__": # 직접 실행할 때
    main()

    