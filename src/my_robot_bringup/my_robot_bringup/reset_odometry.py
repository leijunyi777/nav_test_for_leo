#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty  # 假设重置服务类型是 Empty，如果是其他的请替换

class ResetOdometryTrigger(Node):
    def __init__(self):
        super().__init__('reset_odometry_trigger')
        
        # 1. 创建客户端，服务名假设为 '/reset_odom'，请根据你的小车实际情况修改
        self.service_name = '/reset_odom'
        self.client = self.create_client(Empty, self.service_name)
        
        # 2. 循环等待服务上线（完美解决启动顺序导致调用失败的问题）
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'等待底层服务 {self.service_name} 启动...')
            
        self.req = Empty.Request()

    def send_request(self):
        # 3. 服务上线后，发送异步请求
        self.get_logger().info(f'服务已就绪，正在发送重置里程计请求...')
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    node = ResetOdometryTrigger()
    
    try:
        # 发送请求并等待完成
        node.send_request()
        node.get_logger().info('✅ 里程计重置成功！触发节点即将退出。')
    except Exception as e:
        node.get_logger().error(f'❌ 服务调用失败: {e}')
    finally:
        # 4. 执行完一次后，销毁节点并退出（不占用后台资源）
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()