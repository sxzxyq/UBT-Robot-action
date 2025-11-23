#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hric_msgs.srv import SetMotionNumber

class WaveHandNode(Node):
    def __init__(self):
        super().__init__('wave_hand_node')
        self.client = self.create_client(SetMotionNumber, '/hric/motion/set_motion_number')
        
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务不可用，等待中...')
        
        self.get_logger().info('服务已连接，准备执行挥手动作')
        self.wave_hand()

    def wave_hand(self):
        request = SetMotionNumber.Request()
        request.is_motion = True
        request.motion_number = 1  # 文档中说明1为挥手动作
        
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('挥手动作执行成功！')
            else:
                self.get_logger().error('挥手动作执行失败')
        except Exception as e:
            self.get_logger().error(f'服务调用失败: {str(e)}')
        
        # 完成后关闭节点
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    # 检查是否在正确的环境中运行
    node = Node('env_checker')
    try:
        # 尝试导入所需的消息类型
        from hric_msgs.srv import SetMotionNumber
        node.destroy_node()
        
        # 创建并运行挥手节点
        wave_node = WaveHandNode()
        rclpy.spin(wave_node)
        
    except ImportError:
        node.get_logger().error('无法导入hric_msgs，请确保在正确的ROS2环境中运行')
        node.get_logger().info('请执行: cd ros2ws && source install/setup.bash')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()