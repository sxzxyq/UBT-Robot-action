#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from image_inverter_interfaces.srv import TriggerVerification
import time

def main(args=None):
    rclpy.init(args=args)
    
    # 1. 创建一个极简节点
    node = rclpy.create_node('simple_tester')
    
    # 2. 创建客户端 (不需要任何 callback_group)
    client = node.create_client(TriggerVerification, '/trigger_verification')
    
    print(">>> 正在等待服务上线...")
    if not client.wait_for_service(timeout_sec=5.0):
        print("❌ 服务未上线，退出。")
        node.destroy_node()
        rclpy.shutdown()
        return

    print("✅ 服务在线，准备发送请求...")
    
    # 3. 发送请求
    req = TriggerVerification.Request()
    future = client.call_async(req)
    
    # 4. 【核心】使用 spin_until_future_complete
    # 这就是 ros2 service call 命令行背后的原理
    # 它会阻塞主线程，专门等这个 future 完成
    print(">>> 等待结果中...")
    rclpy.spin_until_future_complete(node, future)
    
    # 5. 获取结果
    if future.result() is not None:
        res = future.result()
        print("="*40)
        print(f"✅ 调用成功！")
        print(f"是否合规: {res.is_compliant}")
        print(f"返回消息: {res.message}")
        print("="*40)
    else:
        print("❌ 调用失败：结果为 None (这通常是不可能的，除非发生异常)")
        print(f"异常信息: {future.exception()}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()