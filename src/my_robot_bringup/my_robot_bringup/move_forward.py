#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoveForwardNode(Node):
    def __init__(self):
        super().__init__('move_forward_node')
        # 创建一个发布者，发布到 'cmd_vel' 话题，消息类型为 Twist，队列长度为 10
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 稍微等待一下，确保发布者和底层的订阅者（如Gazebo或实体车）建立连接
        time.sleep(1.0) 

    def move_and_stop(self):
        msg = Twist()
        
        # 1. 发送前进指令
        # 设定线速度为 0.1 m/s，角速度为 0 (直行)
        msg.linear.x = 0.1
        msg.angular.z = 0.0
        self.get_logger().info('开始前进：速度 0.1 m/s，预计持续 3 秒...')
        self.publisher_.publish(msg)
        
        # 2. 持续运动
        # 0.1 m/s * 3.0 s = 0.3 m
        # 注意：在简单的顺序脚本中可以使用 time.sleep()。
        # 如果节点需要同时处理回调（如订阅传感器），应改用 ROS 2 的 Timer。
        time.sleep(3.0) 
        
        # 3. 速度归零并停止
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('到达目标时间，速度已归零，小车停止。')

def main(args=None):
    rclpy.init(args=args)
    
    # 实例化节点
    node = MoveForwardNode()
    
    # 执行移动和停止的动作
    node.move_and_stop()
    
    # 动作执行完毕后，销毁节点并关闭 ROS 2 接口
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()