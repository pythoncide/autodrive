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


class SelfDrivingNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name,
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.name = name
        self.is_running = True
        self.pid = pid.PID(0.4, 0.0, 0.05)
        self.param_init()

        self.fps = fps.FPS()
        self.image_queue = queue.Queue(maxsize=2)
        self.classes = ['cross_walk', 'light_green', 'light_red', 'light_yellow', 'parking', 'right', 'straight']
        self.display = True
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.colors = common.Colors()
        self.machine_type = os.environ.get('MACHINE_TYPE')
        self.lane_detect = lane_detect.LaneDetector("yellow")

        # publishers
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.servo_state_pub = self.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 1)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)

        # services
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)

        timer_cb_group = ReentrantCallbackGroup()
        self.client = self.create_client(Trigger, '/yolov5_ros2/init_finish')
        self.client.wait_for_service()
        self.start_yolov5_client = self.create_client(Trigger, '/yolov5/start', callback_group=timer_cb_group)
        self.start_yolov5_client.wait_for_service()
        self.stop_yolov5_client = self.create_client(Trigger, '/yolov5/stop', callback_group=timer_cb_group)
        self.stop_yolov5_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()
        self.mecanum_pub.publish(Twist())

        # YOLO는 only_line_follow == false일 때만 실행
        if not self.get_parameter('only_line_follow').value:
            self.send_request(self.start_yolov5_client, Trigger.Request())
        time.sleep(1)

        # start 파라미터와 상관없이 항상 주행 시작
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
        self.slow_down_speed = 0.15 # original 0.1
        self.traffic_signs_status = None
        self.red_loss_count = 0
        self.object_sub = None
        self.image_sub = None
        self.objects_info = []
        self.stuck_count = 0
        self.after_turn = False

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

            # YOLO 구독은 only_line_follow == false일 때만
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

    def shutdown(self, signum, frame):
        self.is_running = False

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(rgb_image)

    def main(self):
        while self.is_running:
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
                cv2.imshow("binary", binary_image)
                cv2.waitKey(1)
                twist = Twist()

                # line following
                result_image, lane_angle, lane_x = self.lane_detect(binary_image, image.copy())
                self.get_logger().info(f'\033[1;32mlane_x: {lane_x}\033[0m')
                self.get_logger().info(f'\033[1;32mlane_angle: {lane_angle}\033[0m')

                if lane_x >= 0 and not self.stop:
                    self.get_logger().info(f'\033[1;32mStart Tracking\033[0m')
                    if lane_x > 270:
                        self.count_turn += 1
                        if self.count_turn > 5:
                            self.get_logger().info(f'\033[1;32mTurn\033[0m')
                            self.count_turn = 0
                            twist.linear.x = self.slow_down_speed
                            twist.angular.z = -0.3
                            self.after_turn = True
                    else:
                        twist.linear.x = self.normal_speed
                        self.count_turn = 0
                        if self.after_turn:
                            self.pid.SetPoint = 130
                            self.pid.update(lane_x)
                            twist.angular.z = common.set_range(self.pid.output, -0.1, 0.1)
                            self.after_turn = False
                    self.mecanum_pub.publish(twist)
                else:
                    self.pid.clear()
                    self.stuck_count += 1
                    if self.stuck_count > 5:
                        twist.linear.x = self.slow_down_speed
                        twist.angular.z = -0.3
                        self.after_turn = True
                    self.mecanum_pub.publish(twist)

                # YOLO 시각화는 only_line_follow == false일 때만
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

    def get_object_callback(self, msg):
        self.objects_info = msg.objects
        if self.objects_info == []:
            self.traffic_signs_status = None
            self.crosswalk_distance = 0
        else:
            min_distance = 0
            for i in self.objects_info:
                class_name = i.class_name
                center = (int((i.box[0] + i.box[2])/2), int((i.box[1] + i.box[3])/2))
                if class_name == 'cross_walk':
                    if center[1] > min_distance:
                        min_distance = center[1]
            self.crosswalk_distance = min_distance


def main():
    node = SelfDrivingNode('self_driving')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()


if __name__ == "__main__":
    main()