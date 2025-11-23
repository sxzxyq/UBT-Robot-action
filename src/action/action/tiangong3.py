#!/usr/bin/env python3

import rclpy
import math
import time
import json
import threading

import numpy as np
from collections import deque

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.timer import Rate

from geometry_msgs.msg import TwistStamped, Twist, Vector3, PointStamped, Point
from std_msgs.msg import Header, String
from hric_msgs.srv import SetMotionMode, SetMotionNumber
from image_inverter_interfaces.srv import TriggerVerification

from rclpy.executors import MultiThreadedExecutor
import threading


class TianGongRobotController(Node):
    def __init__(self):
        super().__init__('tiangong_robot_controller')
        
        # 目标位置相关
        self.target_position = [0.0, 0.0, 0.0]  # 初始化为零
        self.stop_distance = 1.5  # 在距离目标1.5米处停下
        
        # 目标跟踪状态
        self.is_tracking_target = True  # 是否正在接收目标坐标
        self.has_reached_target = False  # 是否已到达目标位置
        self.current_target_id = 0  # 当前目标ID
        self.previous_target_id = 0  # 上一个目标ID
        self.target_stability_threshold = 0.3  # 目标稳定性阈值（米）
        
        # 目标坐标缓冲区
        self.target_buffer = deque(maxlen=5)  # 存储最近5个目标坐标
        self.stable_target = None  # 稳定后的目标坐标
        
        # 语音相关
        self.voice1_path = "/home/nvidia/data/speech/tixing1.mp3"    # 语音1路径
        self.voice2_path = "/home/nvidia/data/speech/thanks.mp3"     # 语音2路径
        self.voice3_path = "/home/nvidia/data/speech/warning.mp3"    # 语音3路径
        
        # 服务质量配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 发布者
        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped, 
            '/hric/robot/cmd_vel', 
            qos_profile
        )
        
        self.voice_publisher = self.create_publisher(
            String,
            '/xunfei/tts_play',
            qos_profile
        )
        
        # 订阅者
        self.target_subscriber = self.create_subscription(
            PointStamped,
            '/helmet/position_filtered_transformed', 
            self.target_callback,
            qos_profile
        )
        
        # 服务客户端
        self.motion_mode_client = self.create_client(
            SetMotionMode, 
            '/hric/motion/set_motion_mode'
        )

        self.motion_number_client = self.create_client(
            SetMotionNumber, 
            '/hric/motion/set_motion_number'
        )

        while not self.motion_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('运动模式服务不可用，等待中...')
        while not self.motion_number_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('动作调用服务不可用，等待中...')
        
        self.get_logger().info("天工行者机器人控制器已初始化")
        self.get_logger().info("等待目标坐标...")

    def target_callback(self, msg):
        """目标坐标回调函数"""
        if not self.is_tracking_target:
            return
            
        # 获取目标坐标
        target_pos = [msg.point.x, msg.point.y, msg.point.z]
        
        # 添加到缓冲区
        self.target_buffer.append(target_pos)
        
        # 检查目标稳定性
        if self.check_target_stability():
            self.stable_target = self.calculate_average_target()
            self.get_logger().info(f"接收到稳定目标: {self.stable_target}")
            
            # 停止接收新坐标，开始执行任务
            self.is_tracking_target = False
            self.current_target_id += 1
            
            # 开始执行任务
            threading.Thread(target=self.execute_complete_task, daemon=True).start()

    ##########目标稳定性判断##########
    def check_target_stability(self):
        """检查目标坐标是否稳定"""
        if len(self.target_buffer) < self.target_buffer.maxlen:
            return False
            
        # 计算所有点之间的最大距离
        max_distance = 0
        for i in range(len(self.target_buffer)):
            for j in range(i+1, len(self.target_buffer)):
                dist = self.calculate_distance(self.target_buffer[i], self.target_buffer[j])
                max_distance = max(max_distance, dist)
                
        return max_distance < self.target_stability_threshold

    def calculate_average_target(self):
        """计算平均目标坐标"""
        if not self.target_buffer:
            return [0.0, 0.0, 0.0]
            
        avg_x = sum(pos[0] for pos in self.target_buffer) / len(self.target_buffer)
        avg_y = sum(pos[1] for pos in self.target_buffer) / len(self.target_buffer)
        avg_z = sum(pos[2] for pos in self.target_buffer) / len(self.target_buffer)
        
        return [avg_x, avg_y, avg_z]

    def calculate_distance(self, pos1, pos2):
        """计算两个位置之间的距离"""
        return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2)
        # return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2 + (pos1[2]-pos2[2])**2)

    def wait_for_target(self, timeout=1.0):
        """等待目标坐标，带超时"""
        self.get_logger().info("等待目标坐标...")
        start_time = time.time()
        
        while (time.time() - start_time < timeout and 
               not self.stable_target and 
               self.is_tracking_target):
            time.sleep(0.1)
            
        if not self.stable_target:
            self.get_logger().warning("等待目标坐标超时")
            return False
        return True
    ##########目标稳定性判断##########
        
    ##########动作和语音指令##########
    def set_motion_mode(self, mode, swing_arm=False):
        """设置运动模式"""
        if not self.motion_mode_client.service_is_ready():
            self.get_logger().error("运动模式服务不可用")
            return False
            
        request = SetMotionMode.Request()
        request.walk_mode_request = mode
        request.is_need_swing_arm = swing_arm
        
        mode_names = {
            1: "僵停",
            2: "回零", 
            3: "站立",
            4: "原地踏步",
            5: "原地跑步"
        }
        
        self.get_logger().info(f"设置运动模式: {mode_names.get(mode, '未知')}")
        
        future = self.motion_mode_client.call_async(request)
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("运动模式设置成功")
                return True
            else:
                self.get_logger().error(f"运动模式设置失败")
                return False
        except Exception as e:
            # self.get_logger().error(f"服务调用异常: {str(e)}")
            return False
    
    def call_motion_number(self, motion_number):
        """调用动作编号"""
        if not self.motion_number_client.service_is_ready():
            self.get_logger().error("动作编号服务不可用")
            return False
            
        request = SetMotionNumber.Request()
        request.is_motion = True
        request.motion_number = motion_number
        
        motion_names = {
            1: "挥手",
            2: "动作2", 
            3: "鞠躬",
            4: "动作4", 
            5: "动作5"
        }
        
        self.get_logger().info(f"执行动作: {motion_names.get(motion_number, '未知')}")
        
        future = self.motion_number_client.call_async(request)
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("动作调用成功")
                return True
            else:
                self.get_logger().error("动作调用失败")
                return False
        except Exception as e:
            # self.get_logger().error(f"服务调用异常: {str(e)}")
            return False
        
    def play_voice(self, audio_file_path):
        """播放语音"""
        voice_msg = String()
        voice_data = {
        "file": audio_file_path
        }
        voice_msg.data = json.dumps(voice_data)
        
        self.voice_publisher.publish(voice_msg)
        # self.get_logger().info(f"播放语音文件: {audio_file_path}")
    ##########动作和语音指令##########
    
    ##########运动指令##########
    def calculate_target_angle_and_distance(self):
        """计算目标角度和距离"""
        if not self.stable_target:
            self.get_logger().error("没有稳定的目标坐标")
            return 0, 0
            
        x, y, z = self.stable_target
        distance = math.sqrt(x**2 + y**2)
        angle = math.atan2(y, x)  # 计算目标相对于机器人的角度
        
        self.get_logger().info(f"目标位置: ({x:.2f}, {y:.2f}, {z:.2f})")
        self.get_logger().info(f"目标距离: {distance:.2f} 米")
        self.get_logger().info(f"目标角度: {math.degrees(angle):.2f} 度")

        return angle, distance
    
    def publish_velocity_command(self, linear_x, angular_z):
        """发布速度控制命令"""
        twist_stamped = TwistStamped()
        
        # 设置header
        twist_stamped.header = Header()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'
        
        # 设置速度
        twist = Twist()
        twist.linear = Vector3(x=linear_x, y=0.0, z=0.0)
        twist.angular = Vector3(x=0.0, y=0.0, z=angular_z)
        
        twist_stamped.twist = twist
        
        self.cmd_vel_publisher.publish(twist_stamped)
        self.get_logger().info(f"发布速度命令: 线速度({linear_x}), 角速度({angular_z})")
    
    def rotate_to_target(self, target_angle, rotation_speed=0.1):
        """旋转到目标方向"""
        self.get_logger().info(f"开始向目标旋转{math.degrees(target_angle):.2f} 度")
        
        # 根据角度确定旋转方向
        if target_angle > 0:
            angular_speed = rotation_speed  # 左转
        else:
            angular_speed = -rotation_speed  # 右转
        
        # 估计旋转时间（简单估算）
        rotation_time = abs(target_angle) / rotation_speed
        self.get_logger().info(f'开始旋转，预计时间: {rotation_time:.2f} 秒')
        
        # 发布旋转命令
        start_time = time.time()
        while time.time() - start_time < rotation_time:
            self.publish_velocity_command(0.0, angular_speed)
            time.sleep(0.05)
        
        # 停止旋转
        self.publish_velocity_command(0.0, 0.0)
        self.get_logger().info("旋转完成")
    
    def move_to_target(self, move_distance, move_speed=0.15):
        """移动到目标距离"""
        self.get_logger().info(f"开始向目标移动 {move_distance:.2f} 米")
        
        # 估计移动时间
        move_time = move_distance / move_speed
        self.get_logger().info(f'开始移动，预计时间: {move_time:.2f} 秒')
        
        # 发布移动命令
        start_time = time.time()
        while time.time() - start_time < move_time:
            self.publish_velocity_command(move_speed, 0.0)
            time.sleep(0.05)
        
        # 停止移动
        self.publish_velocity_command(0.0, 0.0)
        self.get_logger().info("移动完成")
    ##########运动指令##########
    
    # def send_verification_request(self):
    #     """发送请求并阻塞等待结果 (手写等待逻辑版)"""
        
    #     # ================= 手写 wait_for_service 逻辑 =================
    #     timeout = 10.0  # 总超时时间
    #     start_time = time.time()
        
    #     # 循环检查服务是否在线
    #     while not self.verification_client.service_is_ready():
    #         # 检查是否超时
    #         if time.time() - start_time > timeout:
    #             self.get_logger().error(f"❌ 视觉服务未上线 (等待超时 {timeout}s)")
    #             return None
            
    #         # 打印日志提示 (每2秒打印一次，避免刷屏)
    #         if int(time.time() - start_time) % 2 == 0:
    #             self.get_logger().info("正在等待视觉验证服务上线...")
                
    #         time.sleep(0.5) # 稍微睡一下，避免死循环占用CPU
    #     # ============================================================

    #     req = TriggerVerification.Request()
        
    #     # 发送异步请求
    #     future = self.verification_client.call_async(req)
        
    #     try:
    #         # 阻塞等待结果 (依赖主线程的 spin 来接收数据)
    #         return future.result()
    #     except Exception as e:
    #         self.get_logger().error(f"❌ 服务调用异常: {e}")
    #         return None
    
    def motion_and_voice(self):
        """执行动作、播放语音，检查目标状态"""
        
        node = rclpy.create_node('verification_node')
        verification_client = node.create_client(
            TriggerVerification, 
            '/trigger_verification'
        )

        # 执行挥手动作
        self.get_logger().info("执行挥手动作")
        self.call_motion_number(1)  # 1 = 挥手
        time.sleep(3)  # 等待动作完成
            
        # 播放语音1
        self.get_logger().info("播放语音1")
        self.play_voice(self.voice1_path)
        time.sleep(5)

        # self.get_logger().info("等待10秒后检查目标状态")
        # time.sleep(5)
        
        self.get_logger().info("第一次提示完成，正在调用视觉服务进行验证")
        req = TriggerVerification.Request()
        future = verification_client.call_async(req)
        rclpy.spin_until_future_complete(node, future)

        if future.result()==True:
            self.get_logger().info("已佩戴安全帽")

            self.get_logger().info("执行鞠躬动作")
            self.call_motion_number(3)  # 鞠躬
            time.sleep(3)

            self.get_logger().info("播放语音2")
            self.play_voice(self.voice2_path)
            time.sleep(5)

        else:
            self.get_logger().info("未佩戴安全帽")
            
            self.get_logger().info("执行挥手动作")
            self.call_motion_number(1)  # 1 = 挥手
            time.sleep(3)  # 等待动作完成
            
            self.get_logger().info("播放语音1")
            self.play_voice(self.voice1_path)
            time.sleep(5)

            # self.get_logger().info("等待10秒后检查目标状态")
            # time.sleep(5)
                
            # 再次检查目标状态
            self.get_logger().info("第二次提示完成，正在调用视觉服务进行验证")
            req = TriggerVerification.Request()
            future = verification_client.call_async(req)
            rclpy.spin_until_future_complete(node, future)

            if future.result()==True:
                self.get_logger().info("已佩戴安全帽")

                self.get_logger().info("执行鞠躬动作")
                self.call_motion_number(3)  # 鞠躬
                time.sleep(3)

                self.get_logger().info("播放语音2")
                self.play_voice(self.voice2_path)
                time.sleep(5)

            else:
                self.get_logger().info("未佩戴安全帽")
                
                self.get_logger().info("播放语音3")
                self.play_voice(self.voice3_path)
                time.sleep(5)

        node.destroy_node()
    
    def execute_complete_task(self):
        """执行完整任务"""
        try:
            self.get_logger().info("=== 天工行者机器人开始执行任务 ===")
            
            # 设置目标位置为稳定目标
            self.target_position = self.stable_target.copy()
            self.previous_target_id = self.current_target_id
            
            # 1. 切换到原地踏步模式
            self.set_motion_mode(4)  # 4 = 原地踏步
            self.publish_velocity_command(0.0, 0.0)
            time.sleep(1)  # 等待模式切换
            
            # 2. 计算目标角度和距离
            self.get_logger().info("计算目标位置")
            target_angle, target_distance = self.calculate_target_angle_and_distance()
            
            # 3. 旋转到目标方向
            if target_angle != 0.0:
                self.get_logger().info("旋转到目标方向")
                self.rotate_to_target(target_angle)
            else:
                self.get_logger().info("已经在目标方向，无需旋转")
            time.sleep(1)  # 等待旋转稳定
            
            # 4. 移动到距离目标1.5米的位置
            move_distance = target_distance - self.stop_distance
            if move_distance > 0.0:
                self.get_logger().info("移动到目标距离")
                self.move_to_target(move_distance)
            else:
                self.get_logger().info("已经在目标距离，无需移动")
            time.sleep(1)  # 等待移动稳定
            
            # 5. 切换回原地踏步模式
            self.set_motion_mode(4)  # 4 = 原地踏步
            time.sleep(1)
            
            # 6. 切换到站立模式
            self.set_motion_mode(3)  # 3 = 站立
            self.has_reached_target = True
            time.sleep(1)
            
            # 7. 执行动作、播放语音，检查目标状态
            self.motion_and_voice()
            
            self.get_logger().info("=== 天工行者机器人任务执行完成 ===")
            
        except Exception as e:
            self.get_logger().error(f"任务执行出错: {str(e)}")
    ##########执行任务##########

def main(args=None):
    rclpy.init(args=args)
    controller = TianGongRobotController()

    try:
        # 初始等待目标
        # controller.wait_for_target(1.0)
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()