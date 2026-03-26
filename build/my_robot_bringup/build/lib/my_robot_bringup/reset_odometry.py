#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# 1. 修正导入类型为 Trigger
from std_srvs.srv import Trigger 

class ResetOdometryTrigger(Node):
    def __init__(self):
        super().__init__('reset_odometry_trigger')
        
        # 2. 修正服务名称，确保与 ros2 service list 一致
        self.service_name = '/reset_odometry'
        self.client = self.create_client(Trigger, self.service_name)
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'正在等待服务 {self.service_name} 启动...')
            
        self.req = Trigger.Request()

    def send_request(self):
        self.get_logger().info(f'正在请求重置里程计...')
        self.future = self.client.call_async(self.req)
        # 等待结果返回
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    node = ResetOdometryTrigger()
    
    try:
        result = node.send_request()
        # 3. Trigger 类型有返回字段，可以打印出来看具体结果
        if result.success:
            node.get_logger().info(f'✅ 成功: {result.message}')
        else:
            node.get_logger().warn(f'⚠️ 失败: {result.message}')
            
    except Exception as e:
        node.get_logger().error(f'❌ 调用异常: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()