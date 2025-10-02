#!/usr/bin/env python3
# encoding: utf-8
import os
import cv2
import queue
import threading
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from interfaces.msg import ObjectInfo, ObjectsInfo
import yolov5_ros2.fps as fps
from yolov5 import YOLOv5

ros_distribution = os.environ.get("ROS_DISTRO")
package_share_directory = get_package_share_directory('yolov5_ros2')

class YoloV5Ros2(Node):
    def __init__(self):
        super().__init__('yolov5_ros2')
        self.get_logger().info(f"Current ROS 2 distribution: {ros_distribution}")
        self.fps = fps.FPS()

        # ---- 파라미터 ----
        self.declare_parameter("device", "cpu", ParameterDescriptor(
            name="device", description="Compute device selection, default: cpu, options: cuda:0"))
        self.declare_parameter("model", "yolov5n", ParameterDescriptor(
            name="model", description="Default model selection: yolov5n"))
        self.declare_parameter("image_topic", "/ascamera/camera_publisher/rgb0/image", ParameterDescriptor(
            name="image_topic", description="Image topic, default"))
        self.declare_parameter("show_result", False, ParameterDescriptor(
            name="show_result", description="Show detection results"))
        self.declare_parameter("pub_result_img", False, ParameterDescriptor(
            name="pub_result_img", description="Publish detection result images"))
        self.declare_parameter("resize_width", 320, ParameterDescriptor(
            name="resize_width", description="Resize width before YOLO inference"))
        self.declare_parameter("resize_height", 240, ParameterDescriptor(
            name="resize_height", description="Resize height before YOLO inference"))

        # ---- YOLO 모델 로드 ----
        model_path = os.path.join(package_share_directory, "config", self.get_parameter('model').value + ".pt")
        device = self.get_parameter('device').value
        self.yolov5 = YOLOv5(model_path=model_path, device=device)

        # ---- ROS pub/sub ----
        self.bridge = CvBridge()
        self.yolo_result_pub = self.create_publisher(Detection2DArray, "yolo_result", 10)
        self.object_pub = self.create_publisher(ObjectsInfo, '~/object_detect', 1)
        self.result_img_pub = self.create_publisher(Image, "result_img", 1)

        image_topic = self.get_parameter('image_topic').value
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)

        # 서비스
        self.create_service(Trigger, '/yolov5/start', self.start_srv_callback)
        self.create_service(Trigger, '/yolov5/stop', self.stop_srv_callback)
        self.create_service(Trigger, '~/init_finish', self.get_node_state)

        self.show_result = self.get_parameter('show_result').value
        self.pub_result_img = self.get_parameter('pub_result_img').value

        # ---- 스레드 큐 구조 ----
        self.image_queue = queue.Queue(maxsize=2)
        self.running = True
        threading.Thread(target=self.yolo_loop, daemon=True).start()

    def get_node_state(self, request, response):
        response.success = True
        return response

    def start_srv_callback(self, request, response):
        self.get_logger().info("[YOLO] start detect")
        self.running = True
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info("[YOLO] stop detect")
        self.running = False
        response.success = True
        response.message = "stop"
        return response

    # 카메라 콜백 (이미지 수집만)
    def image_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        # 해상도 줄이기
        w = self.get_parameter("resize_width").value
        h = self.get_parameter("resize_height").value
        cv_image = cv2.resize(cv_image, (w, h))

        if not self.image_queue.full():
            self.image_queue.put((cv_image, msg.header))

    # 별도 스레드에서 YOLO 추론
    def yolo_loop(self):
        while rclpy.ok():
            if not self.running:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
            try:
                image, header = self.image_queue.get(timeout=1)
            except queue.Empty:
                continue

            detect_result = self.yolov5.predict(image)
            self.publish_result(image, detect_result, header)

    # 결과 퍼블리시
    def publish_result(self, image, detect_result, header):
        predictions = detect_result.pred[0]
        boxes = predictions[:, :4]
        scores = predictions[:, 4]
        categories = predictions[:, 5]

        result_msg = Detection2DArray()
        result_msg.header = header

        objects_info = []

        for idx in range(len(categories)):
            name = detect_result.names[int(categories[idx])]
            score = float(scores[idx])
            x1, y1, x2, y2 = [int(v) for v in boxes[idx]]

            detection2d = Detection2D()
            detection2d.id = name
            detection2d.bbox.center.position.x = (x1 + x2) / 2.0
            detection2d.bbox.center.position.y = (y1 + y2) / 2.0
            detection2d.bbox.size_x = float(x2 - x1)
            detection2d.bbox.size_y = float(y2 - y1)

            obj_pose = ObjectHypothesisWithPose()
            obj_pose.hypothesis.class_id = name
            obj_pose.hypothesis.score = score
            detection2d.results.append(obj_pose)
            result_msg.detections.append(detection2d)

            object_info = ObjectInfo()
            object_info.class_name = name
            object_info.box = [x1, y1, x2, y2]
            object_info.score = round(score, 2)
            object_info.width = image.shape[1]
            object_info.height = image.shape[0]
            objects_info.append(object_info)

            self.get_logger().info(
                f"[YOLO] {name} ({score:.2f}) box=({x1},{y1},{x2},{y2})"
            )

        # 퍼블리시
        if len(objects_info) > 0:
            object_msg = ObjectsInfo()
            object_msg.objects = objects_info
            self.object_pub.publish(object_msg)
        if len(categories) > 0:
            self.yolo_result_pub.publish(result_msg)

        if self.show_result or self.pub_result_img:
            self.fps.update()
            image = self.fps.show_fps(image)
            if self.show_result:
                cv2.imshow('result', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)
            if self.pub_result_img:
                img_msg = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")
                img_msg.header = header
                self.result_img_pub.publish(img_msg)


def main():
    rclpy.init()
    node = YoloV5Ros2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()