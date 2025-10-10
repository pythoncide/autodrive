#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import lgpio  # RPi.GPIO 대신 lgpio 사용

class LEDController(Node):
    def __init__(self):
        super().__init__('led_controller')

        # === [1] GPIO 핀 설정 ===
        self.declare_parameter('led_pin', 18)
        self.led_pin = self.get_parameter('led_pin').get_parameter_value().integer_value

        # gpiochip0 열기
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, self.led_pin)  # 출력 모드로 설정
        lgpio.gpio_write(self.chip, self.led_pin, 0)      # 초기 상태 OFF

        # === [2] ROS2 구독 설정 ===
        self.subscription = self.create_subscription(
            Bool,
            '/led_cmd',
            self.listener_callback,
            10
        )

        self.get_logger().info(f"LED Controller started (GPIO {self.led_pin})")

    def listener_callback(self, msg: Bool):
        """LED ON/OFF 제어"""
        lgpio.gpio_write(self.chip, self.led_pin, 1 if msg.data else 0)
        state = "ON" if msg.data else "OFF"
        self.get_logger().info(f"💡 LED {state} (GPIO {self.led_pin})")

    def destroy_node(self):
        """종료 시 자원 해제"""
        lgpio.gpio_write(self.chip, self.led_pin, 0)
        lgpio.gpiochip_close(self.chip)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LEDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()