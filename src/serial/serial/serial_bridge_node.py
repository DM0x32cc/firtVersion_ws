#!/usr/bin/env python3
"""
串口协议桥接节点
基于 HC-12 二进制帧协议，负责：
  接收: 小车速度(0x10) → /car_state
       地面站指令(0x13) → /launch, /task
  发送: 飞机状态+位置(0x12) → 地面站
"""

import struct
import time
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from msg_tool.msg import CarState
from geometry_msgs.msg import PoseStamped


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 9600)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.ser = serial.Serial(port, baud, timeout=0.01)
        self.get_logger().info(f'串口已打开: {port} @ {baud}')

        self.buf = bytearray()
        self.HEADER = 0xAA
        self.DEVICE_ID = 0x01  # 飞机

        self.frame_count_ = 0
        self.bad_checksum_ = 0

        # ---------- 发布者 ----------
        self.launch_pub = self.create_publisher(Bool, '/launch', 10)
        self.task_pub = self.create_publisher(Int32, '/task', 10)
        self.car_pub = self.create_publisher(CarState, '/car_state', 10)

        # ---------- 订阅者（用于发送 0x12）----------
        self.latest_pose = None
        self.latest_state = 'INIT'
        self.latest_task_id = 1

        from msg_tool.msg import FlightInfo
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._pose_cb, 10)
        self.task_sub = self.create_subscription(
            FlightInfo, '/task_reply',
            self._task_reply_cb, 10)

        # ---------- 持续发话题（收到串口指令后发 7 秒）----------
        self._pub_task_id = 0                # 正在持续发布的任务号（0=无）
        self._pub_launch_val = False         # 正在持续发布的 launch 值
        self._pub_repeat_count = 0           # 剩余发布次数（70 次 = 7 秒 @10Hz）
        self._repeat_timer = self.create_timer(0.1, self._repeat_pub)  # 10Hz

        # ---------- 定时器 ----------
        self.read_timer = self.create_timer(0.02, self._read_serial)   # 50Hz 读
        self.send_timer = self.create_timer(0.05, self._send_status)   # 20Hz 发

    # ---------- 订阅回调 ----------
    def _pose_cb(self, msg: PoseStamped):
        self.latest_pose = msg

    def _task_reply_cb(self, msg):
        self.latest_state = msg.state
        self.latest_task_id = msg.task_id

    # ---------- 读串口 + 帧解析 ----------
    def _read_serial(self):
        try:
            raw = self.ser.read(self.ser.in_waiting or 1)
        except Exception:
            return
        if not raw:
            return
        self.buf.extend(raw)

        while True:
            idx = self.buf.find(self.HEADER)
            if idx < 0:
                break
            self.buf = self.buf[idx:]

            if len(self.buf) < 6:
                break

            data_len = self.buf[4]
            if data_len > 64:
                self.buf.pop(0)
                continue

            frame_len = 6 + data_len
            if len(self.buf) < frame_len:
                break

            frame = bytes(self.buf[:frame_len])
            self.buf = self.buf[frame_len:]

            if self._checksum(frame[:-1]) != frame[-1]:
                self.bad_checksum_ += 1
                continue

            self._dispatch(frame)

    # ---------- 校验和 ----------
    @staticmethod
    def _checksum(data: bytes) -> int:
        return sum(data) & 0xFF

    # ---------- 帧分发 ----------
    def _dispatch(self, frame: bytes):
        hdr, src, dst, msg_type, data_len = struct.unpack_from('BBBBB', frame)
        payload = frame[5:5 + data_len]
        self.frame_count_ += 1

        if src == self.DEVICE_ID:
            return  # 自己发的，忽略

        self.get_logger().debug(
            f'收到帧: src=0x{src:02X} dst=0x{dst:02X} '
            f'type=0x{msg_type:02X} len={data_len}')

        if msg_type == 0x10:        # 小车速度向量
            self._handle_car_speed(payload)
        elif msg_type == 0x13:      # 地面站指令
            self._handle_command(payload)

    # ---------- 处理 0x10: 小车速度 ----------
    def _handle_car_speed(self, data: bytes):
        if len(data) < 8:
            return
        speed, angle = struct.unpack('!ff', data[:8])
        msg = CarState()
        msg.speed = float(speed)
        msg.deviation_angle = float(angle)
        msg.status = 0
        self.car_pub.publish(msg)

    # ---------- 处理 0x13: 地面站指令 ----------
    def _handle_command(self, data: bytes):
        if len(data) < 1:
            return
        cmd = data[0]
        self.get_logger().info(
            f'收到指令: 0x{cmd:02X} (飞机状态: {self.latest_state}, '
            f'发布中: {self._pub_repeat_count > 0})')

        if cmd in (0x01, 0x02):        # 起飞指令
            if self._pub_repeat_count > 0:
                self.get_logger().debug('已在持续发布中，跳过重复指令')
                return
            if self.latest_state not in ('INIT', 'LAND'):
                self.get_logger().warn(
                    f'飞机状态={self.latest_state}，非 INIT/LAND，忽略起飞指令')
                return
            tid = 1 if cmd == 0x01 else 2
            self._start_repeat_pub(task_id=tid, launch_val=True)
        elif cmd == 0xFF:              # 紧急停止
            self._stop_repeat_pub()
            msg = Bool(data=False)
            self.launch_pub.publish(msg)
            self.get_logger().info('紧急停止，发布 /launch: false')

    # ---------- 持续发布（7 秒，10Hz，共 70 次）----------
    def _start_repeat_pub(self, task_id: int, launch_val: bool):
        self._pub_task_id = task_id
        self._pub_launch_val = launch_val
        self._pub_repeat_count = 70   # 7 秒 @ 10Hz
        self.get_logger().info(f'开始持续发布: task={task_id}, launch={launch_val}，持续 7 秒')

    def _stop_repeat_pub(self):
        self._pub_repeat_count = 0
        self._pub_task_id = 0

    def _repeat_pub(self):
        if self._pub_repeat_count <= 0:
            return
        msg_t = Int32(data=self._pub_task_id)
        self.task_pub.publish(msg_t)
        msg_l = Bool(data=self._pub_launch_val)
        self.launch_pub.publish(msg_l)
        self._pub_repeat_count -= 1
        if self._pub_repeat_count == 0:
            self.get_logger().info('持续发布完成（7 秒）')

    # ---------- 发送 0x12: 飞机状态+位置 ----------
    def _send_status(self):
        pose = self.latest_pose
        if pose is None:
            return

        status_byte = self._build_status_byte()

        try:
            payload = struct.pack(
                '!Bfff',
                status_byte,
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            )
        except Exception:
            return

        data_len = len(payload)
        header = struct.pack('BBBBB',
                             0xAA,           # 帧头
                             self.DEVICE_ID,  # 源: 飞机
                             0x00,           # 目标: 广播
                             0x12,           # 类型: 状态+位置
                             data_len)       # 数据长度

        frame = header + payload
        checksum = self._checksum(frame)
        frame += bytes([checksum])

        try:
            self.ser.write(frame)
        except Exception as e:
            self.get_logger().error(f'串口发送失败: {e}')

    def _build_status_byte(self) -> int:
        val = 0
        # bit 0: 保留 (实际解锁状态需要从 mavros 订阅，这里先填 1)
        val |= 0x01
        # bit 1: 飞行模式 (offboard = 1)
        val |= 0x02
        # bit 3: 任务执行中
        if self.latest_state not in ('INIT', 'LAND', 'UNKNOWN'):
            val |= 0x08
        return val

    def destroy_node(self):
        self.get_logger().info(
            f'统计: 有效帧={self.frame_count_} 校验失败={self.bad_checksum_}')
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
