#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/03/28
# @author:aiden
# autonomous driving

import os
import cv2
import math
import time
now = time.monotonic
import queue
import rclpy
import threading
import numpy as np
import sdk.pid as pid
import sdk.fps as fps
from rclpy.node import Node
import sdk.common as common
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from interfaces.msg import ObjectsInfo
from std_srvs.srv import SetBool, Trigger
from sdk.common import colors, plot_one_box
from example.self_driving import lane_detect
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from ros_robot_controller_msgs.msg import BuzzerState, SetPWMServoState, PWMServoState, RGBStates, RGBState, ButtonState
from std_msgs.msg import Bool, String


class SelfDrivingNode(Node):
    def __init__(self, name):   
        rclpy.init()
        super().__init__(
            name,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        # [1] 기본 속성 초기화

        self.name = name
        self.is_running = True
        self.pid = pid.PID(0.4, 0.0, 0.05)
        self.debug = False
        self.fps = fps.FPS()
        self.image_queue = queue.Queue(maxsize=2)
        self.classes = ['cross_walk', 'light_green', 'light_red', 'light_yellow', 'parking', 'right', 'straight']
        self.display = True
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.colors = common.Colors()
        self.machine_type = os.environ.get('MACHINE_TYPE')
        self.lane_detect = lane_detect.LaneDetector("yellow")

        # self.lane_detect = lane_detect.LaneDetector("yellow") 바로 아래
        self.rois_default = tuple(self.lane_detect.rois)       # 기존 ROI 백업
        r0 = self.rois_default[0]                               # 가장 아래 ROI 하나만
        self.rois_near = ((r0[0], r0[1], r0[2], r0[3], 1.0),)   # 하단 ROI만, 가중치 1.0
        self.after_turn_window = 1.2                            # 턴 직후 1.2초만 하단 ROI 사용
        self.after_turn_ts = None


        # [2] 퍼블리셔를 먼저 생성 (param_init에서 LED 제어 호출함)
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.servo_state_pub = self.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 1)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)
        self.rgb_pub = self.create_publisher(RGBStates, '/ros_robot_controller/set_rgb', 1)
        self.led_pub = self.create_publisher(Bool, '/led_cmd', 10)  # 브레드보드 LED 제어 퍼블리셔
        self.lcd_pub = self.create_publisher(String, 'ui/lcd', 10) # 브레드보드 LCD 제어 퍼블리셔
        self._lcd_last, self._lcd_ts = "", 0.0 # LCD 스로틀 헬퍼
        self.but_sub = self.create_subscription(ButtonState, '/ros_robot_controller/button', self.button_callback, 1)

        # [3] 서비스 생성
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)

        # [4] Yolov5 관련 클라이언트 초기화
        timer_cb_group = ReentrantCallbackGroup()
        self.client = self.create_client(Trigger, '/yolov5_ros2/init_finish')
        self.client.wait_for_service()
        self.start_yolov5_client = self.create_client(Trigger, '/yolov5/start', callback_group=timer_cb_group)
        self.start_yolov5_client.wait_for_service()
        self.stop_yolov5_client = self.create_client(Trigger, '/yolov5/stop', callback_group=timer_cb_group)
        self.stop_yolov5_client.wait_for_service()

        # [5] 파라미터 초기화 (여기서 LED RED 켜짐)
        self.param_init()

        # [6] 타이머 등록 및 초기화 로직 실행
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

        self.get_logger().info('\033[1;32m[SelfDrivingNode Initialized]\033[0m')

    def init_process(self):
        self.timer.cancel()
        self.mecanum_pub.publish(Twist())

        if not self.get_parameter('only_line_follow').value:
            self.send_request(self.start_yolov5_client, Trigger.Request())
        time.sleep(1)

        self.display = True
        self.enter_srv_callback(Trigger.Request(), Trigger.Response())
        request = SetBool.Request()
        request.data = True
        self.set_running_srv_callback(request, SetBool.Response())

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def param_init(self):
        self.start = False
        self.enter = False
        self.stop = False
        self.have_turn_right = False
        self.detect_turn_right = False
        self.detect_far_lane = False
        self.park_x = -1
        self.start_turn_time_stamp = 0
        self.count_turn = 0
        self.start_turn = False
        self.count_right = 0
        self.count_right_miss = 0
        self.turn_right = False
        self.last_park_detect = False
        self.count_park = 0
        self.start_park = False
        self.count_crosswalk = 0
        self.crosswalk_distance = 0
        self.crosswalk_length = 0.1 + 0.3
        self.start_slow_down = False
        self.normal_speed = 0.3
        self.slow_down_speed = 0.14 # original 0.1
        self.traffic_signs_status = None
        self.red_loss_count = 0
        self.object_sub = None
        self.image_sub = None
        self.objects_info = []
        self.stuck_count = 0
        self.after_turn = False

        ### --- 우회전 깜박이를 위한 변수 --- ###
        self.blink_period = 0.5  # LED 깜빡임 주기(초)
        self.last_blink_time = now()
        self.blink_state = False
 
        # self.crosswalk_stop = False
        self.crosswalk_detected = False
        self.crosswalk_stop_start_time = None
        self.crosswalk_cooldown_time = None  
        self.crosswalk_cooldown_duration = 5.0

        # idle → stopping(3s 정지) → cooldown(5s 무시) → idle
        self.cw_state = 'idle'    # 'idle' | 'stopping' | 'cooldown'
        self.cw_ts = None         # 상태 전이 기준 시간 (monotonic)
        self.traffic_signs_detected = False  # 매 프레임 콜백에서 갱신

        # --- [ADD] 우회전 표지판 FSM ---
        # idle → forward(전진 t_fwd) → turning(우회전 t_turn) → idle
        self.right_state = 'idle'        # 'idle' | 'forward' | 'turning'
        self.right_ts = None             # 단계 시작 시각 (monotonic)
        self.right_cnt = 0               # 연속 감지 카운트
        self.right_seen = False          # 프레임 내 관측 플래그
        # 튜닝 파라미터: 로그 4~6회(≈연속 프레임)면 라치
        self.right_on_threshold = 4      # 4 이상에서 라치
        # 동작 시간(초): 코스에 맞게 조정
        self.right_forward_time = 2.00   # 전진 지속
        self.right_turn_time = 2.50      # 우회전 지속
        self.right_turn_angular = -0.60  # 우회전 각속도(좌핸들 기준 음수)

        self.set_rgb_color(255, 0, 0)  # 초기 상태 빨강 (대기/정지)

        # --- PARK FSM (forward -> strafe -> done) ---
        self.park_state = 'idle'        # 'idle' | 'forward' | 'strafe' | 'done'
        self.park_ts = None
        self.park_seen = False
        self.park_cnt = 0
        self.park_on_threshold = 3     # 연속 감지 임계(프레임 수)

        self.park_latch = False

        # 동작 시간/속도 (코스에 맞게 튜닝)
        self.park_forward_time = 5.8    # 전진 시간(s)
        self.park_forward_speed = 0.18
        self.park_strafe_time = 3.0     # y축 이동 시간(s)
        self.park_strafe_speed = 0.18

        # y축 이동 방향: 보통 오른쪽 주차면 -1 (컨트롤러에 따라 반대일 수 있음)
        self.park_y_sign = -1

        # --- PARK BLINK ---
        self.park_blink_period = 0.30   # 색 바뀌는 주기(초)
        self.park_blink_palette = [
            (255, 0, 0),    # R
            (255, 255, 0),  # Y
            (0, 255, 0),    # G
            (0, 255, 255),  # C
            (0, 0, 255),    # B
            (255, 0, 255),  # M
            (255, 255, 255) # W
        ]

        # --- RED LIGHT STOP ---
        self.tl_state = 'idle'      # 'idle' | 'red'
        self.red_seen = False
        self.red_cnt = 0
        self.tl_on_threshold = 5    # 빨간불 연속 감지 프레임 수 (튜닝)
        self.last_red_seen_ts = None
        self.tl_release_gap = 2.0   # 해제: 빨간불이 안 보인 시간(초)
        self.cw_freeze_start = None

        self.tl_min_stop = 1.0
        self.tl_hold_until = None

        # [ADD] Green 라치(연속 프레임) 설정
        self.green_seen = False
        self.green_cnt = 0
        self.tl_green_on_threshold = 1   # 연속 2프레임 초록이면 출발 (필요시 3으로)

        self.last_green_seen_ts = None
        self.green_recent_window = 2.5  # 최근 1.0초 내 초록이면 출발 허용

        self.lx_prev = None
        self.lx_alpha = 0.6   # 0.0~1.0, 클수록 새 값 가중 ↑
        self.lx_jump = 40     # 한 프레임에서 허용할 최대 이동 픽셀

        self.red_ignore_until = None
        self.red_post_right_ignore = 1.0  # 우회전 완료 후 추가 무시 시간(초)

        # STARTUP TL GATE
        self.startup_gate_active = True          # 처음엔 신호 대기
        self.start_green_on_threshold = 1        # 연속 2프레임 초록이면 출발 (필요시 3으로)

        # --- add: prefer CW over TL for a short window ---
        self.last_cw_seen_ts = None
        self.cw_prefer_window = 1.0   # 최근 0.9s 동안은 TL 라치 보류

        # 신호를 처음 보기 시작했을 때 잠깐 TL 라치 지연 (CW 우선)
        self.first_light_seen_ts = None
        self.defer_tl_until_cw_checked = 1.0  # 0.8~1.2s 정도 권장 (로그 gap이 ~0.7s이면 1.0 추천)

        # 버튼 누렸는지 여부
        self.button = True
        self.button_cnt = 0

    def get_node_state(self, request, response):
        response.success = True
        return response

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def enter_srv_callback(self, request, response):    
        self.get_logger().info('\033[1;32m%s\033[0m' % "self driving enter")
        with self.lock:
            self.start = False
            self.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image', self.image_callback, 1)

            if not self.get_parameter('only_line_follow').value:
                self.create_subscription(ObjectsInfo, '/yolov5_ros2/object_detect', self.get_object_callback, 1)

            self.mecanum_pub.publish(Twist())
            self.enter = True
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):     
        self.get_logger().info('\033[1;32m%s\033[0m' % "self driving exit")
        with self.lock:
            self.mecanum_pub.publish(Twist())
        self.param_init()
        response.success = True
        response.message = "exit"
        return response

    def set_running_srv_callback(self, request, response):  
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_running")
        with self.lock:
            self.start = request.data   
            if not self.start:
                self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "set_running"
        return response
    
    # led 색상 제어
    def set_rgb_color(self, r, g, b):
        msg = RGBStates()
        
        # 첫 번째 LED
        led1 = RGBState()
        led1.index = 1
        led1.red = r
        led1.green = g
        led1.blue = b
        msg.states.append(led1)

        # 두 번째 LED
        led2 = RGBState()
        led2.index = 2
        led2.red = r
        led2.green = g
        led2.blue = b
        msg.states.append(led2)

        self.rgb_pub.publish(msg)

    # led 개별 제어
    def set_rgb_dual(self, color1, color2):
        msg = RGBStates()

        led1 = RGBState()
        led1.index = 1
        led1.red, led1.green, led1.blue = color1
        msg.states.append(led1)

        led2 = RGBState()
        led2.index = 2
        led2.red, led2.green, led2.blue = color2
        msg.states.append(led2)

        self.rgb_pub.publish(msg)
    
    def lcd(self, line1: str, line2: str = ""):
        """16x2 LCD용. 같은 내용은 너무 자주 안 보내도록 5Hz(0.2s) 스로틀."""
        msg = f"{line1}\n{line2}".strip()
        ts = now()
        if msg != self._lcd_last or (ts - self._lcd_ts) > 0.2:
            self.lcd_pub.publish(String(data=msg[:128]))  # 과도한 길이 방지
            self._lcd_last, self._lcd_ts = msg, ts

    def blink_all_leds(self, t=None):
        """모든 LED를 팔레트 색상으로 순차 점멸."""
        if t is None:
            t = now()
        idx = int(t / self.park_blink_period) % len(self.park_blink_palette)
        r, g, b = self.park_blink_palette[idx]

        # (A) 양쪽 LED 동일 색상 점멸
        #self.set_rgb_color(r, g, b)

        # (B) 좌우 다른 색으로 번갈아 점멸하고 싶으면 아래 2줄로 교체:
        idx2 = (idx + len(self.park_blink_palette)//2) % len(self.park_blink_palette)
        self.set_rgb_dual(self.park_blink_palette[idx], self.park_blink_palette[idx2])

    def shutdown(self, signum, frame):
        self.is_running = False

    def image_callback(self, ros_image):   
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        if self.image_queue.full():           
            self.image_queue.get()
        self.image_queue.put(cv_image)

    def button_callback(self, msg: ButtonState):
        # msg에는 STM32 보드가 보낸 버튼 상태 데이터가 들어있음
        # self.get_logger().info(f'\033[1;31m{msg.state}\033[0m')
        if msg.state == 1:
            self.get_logger().info(f'\033[1;31mButton {msg.id} 눌림!\033[0m')
            self.button_cnt += 1
            if self.button_cnt % 2 == 1:
                self.button = False
                self.param_init() # 변수 초기화
                self.start = True
            else:
                self.button = True
                self.button_cnt = 0


    def handle_red_light(self):
        """빨간불이면 완전 정지, 빨간불이 사라지면 해제. True 리턴 시 이 프레임은 정지 처리."""
        # 회전(turning) 중이거나 red_ignore_until 그레이스 중일 때만 무시
        if (self.park_state != 'idle') or (self.right_state == 'turning'):
            return False
        
        if (self.red_ignore_until is not None and now() < self.red_ignore_until) and (self.cw_state != 'stopping'):
            return False
        
        # 그레이스가 끝났으면 깔끔히 해제
        if self.red_ignore_until is not None and now() >= self.red_ignore_until:
            self.red_ignore_until = None


        # 진입 조건: 빨간불 연속 감지
        if self.tl_state == 'idle':

            # ▼ 추가: 신호를 막 보기 시작했고(CW 판단 전) 보류 창 내면 TL 라치 보류
            if (self.first_light_seen_ts is not None
                and (now() - self.first_light_seen_ts) < self.defer_tl_until_cw_checked
                and self.cw_state == 'idle'):
                return False

            if self.red_cnt >= self.tl_on_threshold:
                self.tl_state = 'red'
                self.first_light_seen_ts = None
                self.cw_freeze_start = now() if (self.cw_state == 'cooldown') else None
                self.tl_hold_until = now() + self.tl_min_stop
                self.green_cnt = 0
                self.set_rgb_color(255, 0, 0)
                self.led_pub.publish(Bool(data=True))  # 빨간불 정지 시 LED ON
                self.get_logger().info('\033[1;31m[TL] RED → STOP\033[0m')

        # 유지/해제
        if self.tl_state == 'red':
            # [ADD] 최소 정지시간 보장(해제 로직보다 먼저 체크)
            if self.tl_hold_until is not None and now() < self.tl_hold_until:
                self.mecanum_pub.publish(Twist())
                self.set_rgb_color(255, 0, 0)
                return True
            
            can_release = (self.green_cnt >= self.tl_green_on_threshold)

            if can_release:
                # 정지 동안 동결했던 CW 쿨다운 보정
                if self.cw_freeze_start is not None and self.cw_state == 'cooldown' and self.cw_ts is not None:
                    # 빨간불 지속 구간과 '쿨다운 시작 시각'의 겹친 부분만 더한다
                    freeze_end = now()
                    overlap_start = max(self.cw_freeze_start, self.cw_ts)
                    dt = freeze_end - overlap_start
                    if dt > 0:
                        self.cw_ts += dt
                self.cw_freeze_start = None

                self.tl_state = 'idle'
                self.red_cnt = 0
                self.tl_hold_until = None
                self.set_rgb_color(0, 255, 0)
                self.led_pub.publish(Bool(data=False))  # 초록불 출발 시 LED OFF
                self.get_logger().info('\033[1;32m[TL] GO (green)\033[0m')
                # 여기서 정지 퍼블리시는 하지 않음 → 다음 로직이 자연스럽게 출발
            else:
                # 계속 정지
                self.mecanum_pub.publish(Twist())
                self.set_rgb_color(255, 0, 0)
                return True
        return False

    def main(self):
        while self.is_running:
            if self.button:
                time_start = time.time()
                try:    
                    image = self.image_queue.get(block=True, timeout=1)
                except queue.Empty:
                    if not self.is_running:
                        break
                    else:
                        continue

                result_image = image.copy()
                if self.start:
                    binary_image = self.lane_detect.get_binary(image)              
                    #cv2.imshow("binary", binary_image)
                    #cv2.waitKey(1)
                    twist = Twist()

                    # line following
                    result_image, lane_angle, lane_x = self.lane_detect(binary_image, image.copy())
                    # self.get_logger().info(f'\033[1;32mlane_x: {lane_x}\033[0m') # 寃�異쒕맂 李⑥꽑 以묒떖??x醫뚰몴
                    # self.get_logger().info(f'\033[1;32mlane_angle: {lane_angle}\033[0m')    # 李⑥꽑 媛곷룄 (吏꾪뻾 諛⑺뼢)
                    
                    if lane_x is not None and lane_x >= 0:
                        if self.lx_prev is not None:
                            # 큰 점프는 한 프레임에 lx_jump 만큼만 따라가게 클램프
                            delta = lane_x - self.lx_prev
                            if abs(delta) > self.lx_jump:
                                lane_x = int(self.lx_prev + (self.lx_jump if delta > 0 else -self.lx_jump))
                            # EMA로 부드럽게
                            lane_x = int(self.lx_alpha * lane_x + (1 - self.lx_alpha) * self.lx_prev)
                        self.lx_prev = lane_x

                    
                    t = now()

                    if self.startup_gate_active:
                        if self.green_cnt >= self.start_green_on_threshold:
                            self.startup_gate_active = False
                            self.set_rgb_color(0, 255, 0)
                            self.get_logger().info('\033[1;32m[STARTUP] GREEN latched → GO\033[0m')
                            self.lcd("GREEN detacted", "GO")
                            # 게이트 해제 후 아래 기존 주행 로직 진행
                        else:
                            # 아직 출발 불가: 정지 유지
                            twist = Twist()
                            self.mecanum_pub.publish(twist)
                            # LED: 빨간불이 보이면 빨강, 아니면 노랑으로 '대기' 표시
                            if self.red_cnt > 0:
                                self.set_rgb_color(255, 0, 0)
                                self.get_logger().info('\033[1;31m[STARTUP] Waiting: RED detected\033[0m')
                            else:
                                self.set_rgb_color(255, 255, 0)
                                self.get_logger().info('\033[1;33m[STARTUP] Waiting: No GREEN yet\033[0m')
                                self.lcd("No detacted...", "Wait...") # LCD 대기 표시 출력
                            # 이 프레임은 더 진행하지 않음
                            continue


                    if self.after_turn and self.after_turn_ts is not None:
                        if (t - self.after_turn_ts) > self.after_turn_window:
                            self.lane_detect.set_roi(self.rois_default)   # ROI 원복
                            self.after_turn = False
                            self.after_turn_ts = None
                    
                    # (NEW) 빨간불 우선 처리
                    #if self.handle_red_light():
                    #    continue

                    if (self.park_state == 'idle') and (self.right_state == 'idle'):
                        if self.cw_state == 'idle':
                            self.set_rgb_color(0, 255, 0)  # 주행 중(초록)
                            if self.crosswalk_detected:
                                self.cw_state = 'stopping'
                                self.cw_ts = t
                                self.set_rgb_color(255, 0, 0)  # 정지 시작(빨강)
                                self.led_pub.publish(Bool(data=True))  # 횡단보도 정지 시 LED ON
                                self.first_light_seen_ts = None
                                self.get_logger().info('\033[1;35m[CW] STOPPING start (3s)\033[0m')
                                self.lcd("CW detected", "Wait 3 seconds...")

                        elif self.cw_state == 'stopping':
                            # 3초 정지 중에도 신호등 라치 진행(리턴값은 무시)
                            _ = self.handle_red_light()

                            if t - self.cw_ts < 3.0:
                                # 3초 정지
                                twist = Twist()
                                self.mecanum_pub.publish(twist)
                                self.set_rgb_color(255, 0, 0)
                                continue
                            else:
                                seen_light_recent = (
                                    (self.last_red_seen_ts   is not None and (t - self.last_red_seen_ts)   < 2.0) or
                                    (self.last_green_seen_ts is not None and (t - self.last_green_seen_ts) < 2.0)
                                )
                                green_recent = (self.last_green_seen_ts is not None and
                                                (t - self.last_green_seen_ts) < self.green_recent_window)


                                if seen_light_recent:
                                    # 신호 있는 교차로: 빨간만 아니고, 최근에 초록을 봤으면 출발
                                    can_go = (self.tl_state != 'red') and green_recent
                                else:
                                    # 신호 없는 횡단보도: 3초 끝나면 출발 허용
                                    can_go = True

                                if not can_go:
                                    # 빨간불 보이거나(=tl_state=='red') 아직 초록 신뢰 미달이면 계속 정지 유지
                                    twist = Twist()
                                    self.mecanum_pub.publish(twist)
                                    self.set_rgb_color(255, 0, 0)
                                    # stopping 타이머가 흘러 과하게 누적되지 않게, 기준시각 살짝 재설정(선택)
                                    self.cw_ts = t - 3.0
                                    continue
                                else :
                                    # 정지 완료 → 쿨다운 진입
                                    self.cw_state = 'cooldown'
                                    self.cw_ts = t
                                    self.set_rgb_color(0, 255, 0)  # 출발 (초록)
                                    self.led_pub.publish(Bool(data=False))  # 횡단보도 출발 시 LED OFF
                                    # crosswalk_detected 플래그는 콜백에서 다시 갱신됨
                                    self.get_logger().info('\033[1;31m[CW] START IGNORE (cooldown 5s)\033[0m')
                                    self.lcd("[CW] CW pass", "Ignore 5 seconds")

                        elif self.cw_state == 'cooldown':
                            if t - self.cw_ts >= self.crosswalk_cooldown_duration:
                                self.cw_state = 'idle'
                                self.cw_ts = None
                                self.set_rgb_color(0, 255, 0)  # 주행(초록)
                                self.get_logger().info('\033[1;36m[CW] END IGNORE → IDLE\033[0m')
                                self.red_ignore_until = max(self.red_ignore_until or 0.0, now() + 0.8)
                                # 이 프레임 즉시 정지 퍼블리시(튐 방지)
                                self.mecanum_pub.publish(Twist())
                                # 같은 프레임에서 즉시 신호등 평가(빨간불이면 계속 정지)
                                if self.handle_red_light():
                                    continue


                    else:
                        # 파킹 우회전 진행/완료 중엔 CW 감지 자체를 무시
                        self.crosswalk_detected = False

                    if self.cw_state != 'stopping':
                        # 최근에 횡단보도를 봤다면 (cw_prefer_window 안) TL 라치 보류 → CW STOPPING 로그가 먼저 찍히도록
                        if not self.last_cw_seen_ts or (t - self.last_cw_seen_ts) > self.cw_prefer_window:
                            if self.handle_red_light():
                                continue

                    #if (self.cw_state not in 'stopping') and not (self.crosswalk_detected) and (self.handle_red_light()):
                    #    continue


                    if self.cw_state != 'stopping':
                        if self.park_state == 'idle':
                            if self.right_state == 'idle':
                                # 콜백에서 연속 감지로 라치되면 forward 단계로 진입
                                if (self.right_cnt >= self.right_on_threshold) and (self.tl_state != 'red'):
                                    self.right_state = 'forward'
                                    self.right_ts = t
                                    self.crosswalk_detected = False
                                    # 우회전 라치 직후(전진 단계 시작) 빨간불 히스토리/라치 리셋 + 그레이스 시작
                                    self.get_logger().info('\033[1;36m[RIGHT] FORWARD start\033[0m')
                                    self.lcd("RIGHT detected", "Forward")
                            elif self.right_state == 'forward':
                                if t - self.right_ts < self.right_forward_time:
                                    twist = Twist()
                                    if lane_x >= 0 and not self.stop:
                                        # 보정값 (좌표 차이)
                                        error = lane_x - 120  # lane_x가 120보다 크면 오른쪽 → 좌회전 필요
                                        k = 0.003             # 비례 상수 (튜닝 필요)
                                        twist.linear.x = self.normal_speed
                                        twist.angular.z = -k * error  # error > 0 → 왼쪽으로 회전 (부호 맞게 튜닝)
                                        self.mecanum_pub.publish(twist)

                                    else:
                                        self.pid.clear()
                                        self.stuck_count += 1
                                        if self.stuck_count > 5:
                                            twist.linear.x = self.slow_down_speed
                                            twist.angular.z = -0.6
                                            self.after_turn = True
                                        self.mecanum_pub.publish(twist)
                                    continue
                                else:
                                    self.right_state = 'turning'
                                    self.right_ts = t
                                    self.red_ignore_until = now() + self.right_turn_time + 0.2
                                    self.get_logger().info('\033[1;36m[RIGHT] TURNING start\033[0m')
                                    self.lcd("RIGHT start", "Turning...")
                            elif self.right_state == 'turning':
                                if t - self.right_ts < self.right_turn_time:
                                    twist = Twist()
                                    twist.linear.x = self.slow_down_speed
                                    twist.angular.z = self.right_turn_angular
                                    self.mecanum_pub.publish(twist)

                                    # 우회전 중 LED 깜빡이기 (0.3초 간격)
                                    if int((t * 2) % 2) == 0:  # 약 0.5초 단위 토글
                                        self.set_rgb_dual((0, 255, 0), (255, 255, 0))  # 왼쪽=초록, 오른쪽=노랑
                                    else:
                                        self.set_rgb_dual((0, 255, 0), (0, 0, 0))      # 오른쪽 OFF
                                    continue
                                else:
                                    # 시퀀스 종료 및 리셋
                                    self.right_state = 'idle'
                                    self.right_ts = None
                                    self.right_cnt = 0
                                    self.cw_state = 'cooldown'
                                    self.cw_ts = t
                                    self.crosswalk_detected = False

                                    self.after_turn = True
                                    self.after_turn_ts = t
                                    self.lane_detect.set_roi(self.rois_near)
                                    self.red_ignore_until = now() + self.red_post_right_ignore
                                    # [ADD] LED 블링크 확실히 종료
                                    self.blink_state = False
                                    self.last_blink_time = now()      # (선택) 주기 기준 재설정

                                    self.get_logger().info(
                                        f'\033[1;31m[CW] START IGNORE after RIGHT (cooldown {self.crosswalk_cooldown_duration:.1f}s)\033[0m'
                                    )
                                    self.set_rgb_color(0, 255, 0)  # 주행 (초록)
                                    self.get_logger().info('\033[1;36m[RIGHT] DONE → IDLE\033[0m')
                                    self.lcd("RIGHT completed", "Lanedetect start")

                        # === PARK FSM 시작 ===
                        if self.park_state == 'idle':
                            if self.park_latch:
                                self.park_state = 'forward'
                                self.park_ts = t
                                self._park_align_good = 0
                                self.stuck_count = 0
                                # 우회전 라치 초기화 (경합 방지)
                                self.right_cnt = 0
                                self.right_state = 'idle'
                                self.park_cnt = 0
                                self.park_latch = False # 라치 해제
                                self.cw_state = 'idle'          # CW STOP 상태가 남아 파킹 FSM 막는 것 방지
                                self.cw_ts = None
                                self.crosswalk_detected = False
                                self.get_logger().info('\033[1;35m[PARK] FORWARD start\033[0m')
                                self.lcd("PARK detected", "Forward")
                        
                        elif self.park_state == 'forward':
                            if t - self.park_ts < self.park_forward_time:
                                twist = Twist()
                                twist.linear.x = self.park_forward_speed
                                twist.angular.z = 0.0
                                self.mecanum_pub.publish(twist)
                                continue
                            else:
                                self.park_state = 'strafe'
                                self.park_ts = t
                                self.get_logger().info('\033[1;35m[PARK] STRAFE start\033[0m')
                                self.lcd("PARK start", "Parking...")

                        elif self.park_state == 'strafe':
                            if t - self.park_ts < self.park_strafe_time:
                                twist = Twist()
                                twist.linear.x = 0.0
                                twist.linear.y = self.park_y_sign * self.park_strafe_speed
                                twist.angular.z = 0.0
                                self.mecanum_pub.publish(twist)
                                continue
                            else:
                                self.park_state = 'done'
                                self.park_ts = None
                                self.get_logger().info('\033[1;35m[PARK] DONE\033[0m')
                                self.lcd("PARK complete!", "Good Bye!")

                        elif self.park_state == 'done':
                            # 완전 정지 후 주행 비활성화
                            twist = Twist()                  # 0,0,0
                            self.mecanum_pub.publish(twist)
                            self.start = False               # 차선 추종/우회전 FSM 모두 비활성화
                            self.park_cnt = 0 # 라치에 의한 재 트리거 방지
                            self.right_cnt = 0
                            self.get_logger().info('\033[1;35m[PARK] DONE → HOLD\033[0m')
                            # 여기서 1회 점멸 호출(이 프레임)
                            self.blink_all_leds(t)
                            continue                         # 아래 P제어 등 스킵해서 정지 유지

                        # === PARK FSM 끝 ===

                    if lane_x >= 0 and not self.stop:
                        # self.get_logger().info(f'\033[1;32mStart Tracking\033[0m')
                        if lane_x > 270:
                            self.count_turn += 1
                            if self.count_turn > 5:
                                self.count_turn = 0
                                twist.linear.x = self.slow_down_speed
                                twist.angular.z = -0.6
                                self.after_turn = True
                                self.mecanum_pub.publish(twist)
                                
                        else:
                            twist.linear.x = self.normal_speed
                            self.count_turn = 0
                            if lane_x >= 0 and not self.stop:
                                # 보정값 (좌표 차이)
                                error = lane_x - 120  # lane_x가 130보다 크면 오른쪽 → 좌회전 필요
                                k = 0.003             # 비례 상수 (튜닝 필요)
                                twist.linear.x = self.normal_speed
                                twist.angular.z = -k * error  # error > 0 → 왼쪽으로 회전 (부호 맞게 튜닝)
                                
                                # LED: 초록색 (단, 빨간불·횡단보도·주차·우회전 중엔 덮어쓰지 않음) 쿨다운 중에도 초록 유지 (빨간불/파킹/우회전 중만 제외)
                                if (self.tl_state != 'red'
                                    and self.park_state == 'idle'
                                    and self.right_state == 'idle'):
                                    self.set_rgb_color(0, 255, 0)

                                self.mecanum_pub.publish(twist)
                    else:
                        self.pid.clear()
                        self.stuck_count += 1
                        if self.stuck_count > 5:
                            twist.linear.x = self.slow_down_speed
                            twist.angular.z = -0.6
                            self.after_turn = True

                            # 8프레임 이상이면 '실제 회전'으로 간주
                            if self.stuck_count >= 8:
                            #    self.get_logger().info(f'\033[1;33m[LANE LOST] turning to recover lane (frame={self.stuck_count})\033[0m')

                                # 깜빡이 시작
                                if t - self.last_blink_time > self.blink_period:
                                    self.blink_state = not self.blink_state
                                    self.last_blink_time = t

                                if self.blink_state:
                                    self.set_rgb_dual((0, 255, 0), (255, 255, 0))  # 오른쪽 노란색 켜기
                                else:
                                    self.set_rgb_dual((0, 255, 0), (0, 0, 0)) 

                            # 회전 중엔 리셋 타이머 초기화
                            self.blink_reset_ts = t
                        else:
                            # 차선 복귀 시 LED 초기화
                            if lane_x >= 0 and self.blink_state:
                                self.blink_state = False
                                self.set_rgb_color(0, 255, 0)
                            elif lane_x >= 0 and not self.blink_state:
                                self.set_rgb_color(0, 255, 0)
                        self.mecanum_pub.publish(twist)

                    if not self.get_parameter('only_line_follow').value and self.objects_info:
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
                    if self.park_state == 'done':
                        self.blink_all_leds(now())
                    time.sleep(0.01)

                bgr_image = result_image
                if self.display:
                    self.fps.update()
                    bgr_image = self.fps.show_fps(bgr_image)

                self.result_publisher.publish(self.bridge.cv2_to_imgmsg(bgr_image, "bgr8"))
                time_d = 0.03 - (time.time() - time_start)
                if time_d > 0:
                    time.sleep(time_d)
        self.mecanum_pub.publish(Twist())
        rclpy.shutdown()

    def get_object_callback(self, msg):  
        self.objects_info = msg.objects
        # --- [CHG] 프레임 시작 시 상태 초기화 ---
        self.crosswalk_detected = False
        self.crosswalk_distance = 0
        self.traffic_signs_detected = False  # (다른 신호/표지 플래그도 여기서 리셋)
        self.right_seen = False
        self.park_seen = False
        self.red_seen = False
        self.green_seen = False 
        # 쿨다운 활성 여부(FSM 기준). 콜백에서는 cross_walk만 무시하고 나머지는 처리.
        ignore_crosswalk = (
            (self.cw_state == 'cooldown')
            or (self.park_state != 'idle')
            or (self.right_state != 'idle')
        )

        if not self.objects_info:
            self.traffic_signs_status = None
            return

        max_score = 0
        cw_best_center_y = 0
        frame_width = 640
        frame_height = 480

        for obj in self.objects_info:
            if obj.class_name == 'cross_walk':
                if ignore_crosswalk:
                    continue
                score = obj.score
                x1, y1, x2, y2 = obj.box
                width = x2 - x1
                height = y2 - y1
                aspect_ratio = width / (height + 1e-5)
                center_y = int((y1 + y2) / 2)
                # self.red_ignore_until = max(self.red_ignore_until or 0.0, now() + 0.6)

                if score > 0.5 and y2 > frame_height * 0.68 and aspect_ratio > 2.0:
                    if score > max_score:
                        max_score = score
                        self.crosswalk_detected = True
                        cw_best_center_y = center_y
                        self.last_cw_seen_ts = now()  # 최근에 횡단보도 봤다 표시

                        # ▼ 추가: CW가 보였으면 TL 라치 잠깐 보류 + "처음 신호 본 시각" 초기화
                        self.first_light_seen_ts = None
                        if self.cw_state == 'idle':
                            self.red_ignore_until = now() + self.defer_tl_until_cw_checked
                    
                        self.get_logger().info(f'\033[1;32m[CW] Score: {score} y2: {y2}\033[0m')

            if obj.class_name == 'right':
                score = obj.score
                x1, y1, x2, y2 = obj.box
                width = x2 - x1
                height = y2 - y1
                center_y = int((y1 + y2) / 2)

                if score > 0.1 and y2 > frame_height * 0.1:
                    self.right_seen = True
                    self.get_logger().info(f'\033[1;36m[RIGHT]Score: {score} y2: {y2}\033[0m')
            
            if obj.class_name == 'parking':
                score = obj.score
                x1, y1, x2, y2 = obj.box
                width = x2 - x1
                height = y2 - y1
                center_y = int((y1 + y2) / 2)

                if score > 0.1 and y2 > frame_height * 0.1:
                    self.park_seen = True   # <-- 추가: 파킹 관측 라치용
                    self.get_logger().info(f'\033[1;35m[PARK]Score: {score} y2: {y2}\033[0m')

            if obj.class_name == 'light_red':
                score = obj.score
                x1, y1, x2, y2 = obj.box
                width = x2 - x1
                height = y2 - y1
                center_y = int((y1 + y2) / 2)

                if score > 0.1 and y2 > frame_height * 0.1:
                    if (self.right_state == 'turning') or (self.park_state != 'idle') or (self.cw_state == 'cooldown') or (self.red_ignore_until is not None and now() < self.red_ignore_until):
                        pass  # 필요하면 플래그만 남겨도 됨
                    else:
                        self.red_seen = True
                        self.last_red_seen_ts = now()    # ★ 마지막으로 빨간불 본 시각 기록

                        # ▼ 추가: 신호를 "처음 보기 시작"한 시각 (CW 판단할 여유를 주기 위해)
                        if self.first_light_seen_ts is None and self.cw_state == 'idle':
                            self.first_light_seen_ts = now()
                        self.get_logger().info(f'\033[1;35m[RED]Score: {score} y2: {y2}\033[0m')
            
            if obj.class_name == 'light_green':
                score = obj.score
                x1, y1, x2, y2 = obj.box
                if score > 0.1 and y2 > frame_height * 0.1:
                    self.green_seen = True
                    self.last_green_seen_ts = now()   # 추가

                    # ▼ 추가: 신호를 "처음 보기 시작"한 시각 (CW 판단할 여유를 주기 위해)
                    if self.first_light_seen_ts is None and self.cw_state == 'idle':
                        self.first_light_seen_ts = now()
                    self.get_logger().info(f'\033[1;32m[GREEN]Score: {score} y2: {y2}\033[0m')

        self.crosswalk_distance = cw_best_center_y if self.crosswalk_detected else 0

        # 빨간불 히스테리시스
        self.red_cnt = self.red_cnt + 1 if self.red_seen else max(0, self.red_cnt - 1)

        # 초록불 히스테리시스
        self.green_cnt = self.green_cnt + 1 if self.green_seen else max(0, self.green_cnt - 1)

        # --- [ADD] 우회전 연속 감지 카운터(히스테리시스) ---
        # 관측되면 +1, 아니면 1씩 감소(0 하한)
        if self.right_seen:
            self.right_cnt += 1
        else:
            self.right_cnt = max(0, self.right_cnt - 1)

        # --- [ADD] 파킹 연속 감지 카운터 ---
        if self.park_seen:
            self.park_cnt += 1
        else:
            self.park_cnt = max(0, self.park_cnt - 1)

        # ★ 임계 통과 즉시 라치(정지 중이지만 않으면 OK)
        if (not self.park_latch
            and self.park_state == 'idle'
            and self.right_state == 'idle'
            and self.cw_state != 'stopping'
            and self.park_cnt >= self.park_on_threshold):
            self.park_latch = True

def main():
    node = SelfDrivingNode('self_driving')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()


if __name__ == "__main__":
    main()