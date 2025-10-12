#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time, textwrap
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import lgpio  # sudo apt-get install -y python3-lgpio
except Exception as e:
    lgpio = None

CMD, CHR = 0, 1

class HD44780GPIO:
    """QAPASS 1602A (HD44780) 4-bit 모드: RS,E,D4..D7만 사용, RW는 GND 고정."""
    def __init__(self, rs, e, d4, d5, d6, d7, chip=0):
        if lgpio is None:
            raise RuntimeError("python3-lgpio가 필요합니다: sudo apt-get install -y python3-lgpio")
        self.rs, self.e, self.d4, self.d5, self.d6, self.d7 = rs, e, d4, d5, d6, d7
        self.h = lgpio.gpiochip_open(chip)
        for p in (rs, e, d4, d5, d6, d7):
            lgpio.gpio_claim_output(self.h, p)
        time.sleep(0.05)
        # 4-bit 초기화 시퀀스
        self._write4(CMD, 0x33); self._write4(CMD, 0x32)
        self._write4(CMD, 0x28)  # 4-bit, 2-line, 5x8
        self._write4(CMD, 0x0C)  # display on, cursor off
        self._write4(CMD, 0x06)  # entry mode
        self.clear()

    def _pulse(self):
        lgpio.gpio_write(self.h, self.e, 1); time.sleep(0.0005)
        lgpio.gpio_write(self.h, self.e, 0); time.sleep(0.0001)

    def _nibble(self, val):
        lgpio.gpio_write(self.h, self.d4, (val >> 0) & 1)
        lgpio.gpio_write(self.h, self.d5, (val >> 1) & 1)
        lgpio.gpio_write(self.h, self.d6, (val >> 2) & 1)
        lgpio.gpio_write(self.h, self.d7, (val >> 3) & 1)
        self._pulse()

    def _write4(self, mode, byte):
        lgpio.gpio_write(self.h, self.rs, 1 if mode == CHR else 0)
        self._nibble((byte >> 4) & 0x0F)
        self._nibble(byte & 0x0F)

    def cmd(self, c): self._write4(CMD, c)
    def putc(self, ch): self._write4(CHR, ch if isinstance(ch, int) else ord(ch))

    def clear(self):
        self.cmd(0x01); time.sleep(0.002)

    def set_line(self, idx):
        self.cmd(0x80 if idx == 0 else 0xC0)

    def puts(self, text, line=0, width=16):
        self.set_line(line)
        s = text[:width].ljust(width)
        for ch in s:
            self.putc(ch)

    def close(self):
        try:
            for p in (self.rs, self.e, self.d4, self.d5, self.d6, self.d7):
                lgpio.gpio_free(self.h, p)
        finally:
            lgpio.gpiochip_close(self.h)

class LCDController(Node):
    """
    /ui/lcd (std_msgs/String) 구독:
      - "1행\n2행" 형식 권장
      - 줄바꿈 없으면 16자 기준으로 자동 랩
    """
    def __init__(self):
        super().__init__('lcd_controller')

        # 핀 파라미터(BCM 번호). 필요하면 launch에서 바꿔줘.
        self.declare_parameter('rs', 26)
        self.declare_parameter('e', 19)
        self.declare_parameter('d4', 13)
        self.declare_parameter('d5', 6)
        self.declare_parameter('d6', 5)
        self.declare_parameter('d7', 11)
        self.declare_parameter('width', 16)
        self.declare_parameter('topic', 'ui/lcd')
        self.declare_parameter('gpiochip', 0)

        rs = self.get_parameter('rs').value
        e  = self.get_parameter('e').value
        d4 = self.get_parameter('d4').value
        d5 = self.get_parameter('d5').value
        d6 = self.get_parameter('d6').value
        d7 = self.get_parameter('d7').value
        w  = self.get_parameter('width').value
        chip = self.get_parameter('gpiochip').value
        self.width = int(w)

        self.lcd = HD44780GPIO(rs, e, d4, d5, d6, d7, chip=chip)
        self.lcd.puts("SelfDriving UI", 0, self.width)
        self.lcd.puts("LCD Ready", 1, self.width)

        topic = self.get_parameter('topic').value
        self.create_subscription(String, topic, self.on_msg, 10)
        self.get_logger().info(f"LCD subscribe: {topic}")

    def on_msg(self, msg: String):
        text = (msg.data or "").replace("\r", "")
        parts = text.split("\n", 1)
        line1 = parts[0].strip()
        line2 = parts[1].strip() if len(parts) > 1 else ""

        if not line2 and len(line1) > self.width:
            wrapped = textwrap.wrap(line1, self.width, break_long_words=True)
            line1 = wrapped[0] if wrapped else ""
            line2 = wrapped[1] if len(wrapped) > 1 else ""

        self.lcd.puts(line1, 0, self.width)
        self.lcd.puts(line2, 1, self.width)

    def destroy_node(self):
        self.lcd.clear()
        self.lcd.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LCDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()