#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Subscriber(Node):
    def __init__(self):
        # TODO: Implement node initialization
        super().__init__('subscriber_node')
        # TODO: Register callback
        self.subscription = self.create_subscription(
            String,     # 메시지 타입
            'chatter',  # 구독할 토픽 이름
            self.listener_callback,     # 메시지가 들어올 때 실행할 함수
            10  # 큐 사이즈(메시지 버퍼 크기)
        )
        self.subscription

    # 메시지가 들어올 때마다 실행
    def listener_callback(self, msg):
        # TODO: Implemnt a callback
        self.get_logger().info(f'I heard: "{msg.data}"')    # ROS2 로그 출력


def main(args=None):
    rclpy.init(args=args)       # ROS2 초기화
    node = Subscriber()         # Subscriber 노드 생성
    rclpy.spin(node)            # 노드를 계속 실행 (콜백 기다림)
    node.destroy_node()         # 종료 시 노드 삭제
    rclpy.shutdown()            # ROS2 종료


if __name__ == "__main__":
    main()