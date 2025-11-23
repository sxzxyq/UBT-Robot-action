#!/usr/bin/env python3

import rclpy
import math
import time
import json
import numpy as np
from collections import deque

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import TwistStamped, Twist, Vector3, PointStamped
from std_msgs.msg import Header, String
from hric_msgs.srv import SetMotionMode, SetMotionNumber
from image_inverter_interfaces.srv import TriggerVerification

class TianGongRobotController(Node):
    def __init__(self):
        super().__init__('tiangong_robot_controller')
        
        # ================= 原有参数保持不变 =================
        # 目标位置相关
        self.target_position = [0.0, 0.0, 0.0]
        self.stop_distance = 1.5
        
        # 目标跟踪状态
        self.is_tracking_target = True
        self.has_reached_target = False
        self.current_target_id = 0
        self.previous_target_id = 0
        self.target_stability_threshold = 0.3
        
        # 目标坐标缓冲区
        self.target_buffer = deque(maxlen=5)
        self.stable_target = None
        
        # 语音相关
        self.voice1_path = "/home/nvidia/data/speech/tixing1.mp3"
        self.voice2_path = "/home/nvidia/data/speech/thanks.mp3"
        self.voice3_path = "/home/nvidia/data/speech/warning.mp3"
        
        # ================= 通信初始化 =================
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 发布者
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, '/hric/robot/cmd_vel', qos_profile)
        self.voice_publisher = self.create_publisher(String, '/xunfei/tts_play', qos_profile)
        
        # 订阅者
        self.target_subscriber = self.create_subscription(
            PointStamped, '/helmet/position_filtered_transformed', self.target_callback, qos_profile)
        
        # 服务客户端
        self.motion_mode_client = self.create_client(SetMotionMode, '/hric/motion/set_motion_mode')
        self.motion_number_client = self.create_client(SetMotionNumber, '/hric/motion/set_motion_number')
        self.verification_client = self.create_client(TriggerVerification, '/trigger_verification')

        # 等待基础服务
        while not self.motion_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('运动模式服务不可用，等待中...')
        while not self.motion_number_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('动作调用服务不可用，等待中...')
        
        self.get_logger().info("✅ 控制器初始化完成，等待目标坐标...")

    # ================= 1. 目标处理 (保留了你的稳定性算法) =================

    def target_callback(self, msg):
        """只负责接收数据，不再启动线程"""
        if not self.is_tracking_target: return
            
        target_pos = [msg.point.x, msg.point.y, msg.point.z]
        self.target_buffer.append(target_pos)
        
        # 检查目标稳定性 (调用你原来的算法)
        if self.check_target_stability():
            self.stable_target = self.calculate_average_target()
            self.get_logger().info(f"✅ 锁定稳定目标: {self.stable_target}")
            self.is_tracking_target = False # 锁定目标，停止更新

    def wait_for_target_sync(self, timeout=300.0):
        """【手动挡】同步等待目标"""
        self.get_logger().info("正在扫描目标...")
        start_time = time.time()
        
        while rclpy.ok():
            # 【关键】手动 spin_once，让 target_callback 有机会执行
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.stable_target is not None:
                return True
            
            if time.time() - start_time > timeout:
                self.get_logger().warn("等待目标超时")
                return False

    # ================= 2. 服务调用封装 (修复版) =================

    def send_verification_request_sync(self):
        """同步调用视觉服务"""
        if not self.verification_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("视觉服务未上线")
            return None

        req = TriggerVerification.Request()
        future = self.verification_client.call_async(req)
        
        self.get_logger().info(">>> 发起视觉验证，等待结果...")
        
        # 【核心】阻塞并 spin，直到收到结果
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error(f"服务调用异常: {future.exception()}")
            return None

    def set_motion_mode_sync(self, mode):
        """同步设置运动模式"""
        req = SetMotionMode.Request()
        req.walk_mode_request = mode
        future = self.motion_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f"模式设置: {mode}")
            return future.result().success
        return False

    def call_motion_number_sync(self, num):
        """同步调用动作"""
        req = SetMotionNumber.Request()
        req.is_motion = True
        req.motion_number = num
        future = self.motion_number_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f"执行动作: {num}")
            return future.result().success
        return False

    # ================= 3. 业务逻辑 (整合版) =================

    def execute_complete_task(self):
        """执行完整任务"""
        try:
            self.get_logger().info("=== 开始执行任务 ===")
            self.target_position = self.stable_target.copy()
            
            # 1. 切换模式 - 原地踏步
            self.set_motion_mode_sync(4) 
            self.publish_velocity_command(0.0, 0.0)
            time.sleep(1)
            
            # 2. 计算 & 旋转
            target_angle, target_distance = self.calculate_target_angle_and_distance()
            if target_angle != 0.0:
                self.rotate_to_target(target_angle)
            time.sleep(1)
            
            # 3. 移动
            move_distance = target_distance - self.stop_distance
            if move_distance > 0.0:
                self.move_to_target(move_distance)
            time.sleep(1)
            
            # 4. 停止 & 站立
            self.set_motion_mode_sync(4)
            time.sleep(1)
            self.set_motion_mode_sync(3)
            time.sleep(1)
            
            # 5. 交互
            self.motion_and_voice()
            
            self.get_logger().info("=== 任务完成 ===")
            
        except Exception as e:
            self.get_logger().error(f"任务出错: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def motion_and_voice(self):
        """交互逻辑"""
        # --- 第1轮 ---
        self.get_logger().info("[1] 挥手...")
        self.call_motion_number_sync(1)
        time.sleep(3)
        
        self.get_logger().info("[1] 语音...")
        self.play_voice(self.voice1_path)
        time.sleep(15)
        
        # 调用服务
        response = self.send_verification_request_sync()
        if response is None: return

        if response.is_compliant:
            self.handle_success()
            return

        # --- 第2轮 ---
        self.get_logger().info("❌ 未佩戴 -> 第2轮")
        self.call_motion_number_sync(1)
        time.sleep(3)
        self.play_voice(self.voice1_path)
        time.sleep(15)
        
        response = self.send_verification_request_sync()
        if response is None: return

        if response.is_compliant:
            self.handle_success()
        else:
            self.get_logger().info("❌ 仍未佩戴 -> 警告")
            self.play_voice(self.voice3_path)
            time.sleep(5)

    def handle_success(self):
        self.get_logger().info("✅ 已佩戴 -> 感谢")
        self.call_motion_number_sync(3) # 鞠躬
        time.sleep(3)
        self.play_voice(self.voice2_path)
        time.sleep(5)

    # ================= 4. 工具函数 (你的原始逻辑) =================

    def check_target_stability(self):
        if len(self.target_buffer) < self.target_buffer.maxlen: return False
        max_dist = 0
        for i in range(len(self.target_buffer)):
            for j in range(i+1, len(self.target_buffer)):
                d = self.calculate_distance(self.target_buffer[i], self.target_buffer[j])
                max_dist = max(max_dist, d)
        return max_dist < self.target_stability_threshold

    def calculate_average_target(self):
        if not self.target_buffer: return [0.0, 0.0, 0.0]
        return np.mean(self.target_buffer, axis=0).tolist()

    def calculate_distance(self, pos1, pos2):
        return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2)

    def calculate_target_angle_and_distance(self):
        x, y, z = self.stable_target
        dist = math.sqrt(x**2 + y**2)
        angle = math.atan2(y, x)
        return angle, dist

    def publish_velocity_command(self, lx, az):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'base_link'
        ts.twist.linear.x = lx
        ts.twist.angular.z = az
        self.cmd_vel_publisher.publish(ts)

    def rotate_to_target(self, angle):
        speed = 0.1 if angle > 0 else -0.1
        duration = abs(angle) / 0.1
        start = time.time()
        while time.time() - start < duration:
            self.publish_velocity_command(0.0, speed)
            time.sleep(0.05)
        self.publish_velocity_command(0.0, 0.0)

    def move_to_target(self, dist):
        duration = dist / 0.15
        start = time.time()
        while time.time() - start < duration:
            self.publish_velocity_command(0.15, 0.0)
            time.sleep(0.05)
        self.publish_velocity_command(0.0, 0.0)

    def play_voice(self, path):
        msg = String()
        msg.data = json.dumps({"file": path})
        self.voice_publisher.publish(msg)

# ================= 5. 主程序 (无多线程，无死锁) =================
def main(args=None):
    rclpy.init(args=args)
    controller = TianGongRobotController()

    try:
        # 1. 循环等待直到拿到稳定目标
        if controller.wait_for_target_sync(timeout=120.0):
            
            # 2. 执行任务 (按顺序走，调服务时会自动 spin)
            controller.execute_complete_task()
            
        else:
            print("超时未检测到目标，程序退出")

    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()