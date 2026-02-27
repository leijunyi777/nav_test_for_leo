"""
=======================================================================================
Robot Navigation Microservice - Real Robot Deployment (Action Client Version)
=======================================================================================
Design Philosophy:
    Uses standard ROS 2 ActionClient to communicate with Nav2. 
    [REAL ROBOT MODE]: use_sim_time is strictly set to FALSE.
=======================================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import random
import math

from rclpy.parameter import Parameter

class NavControllerNode(Node):
    def __init__(self):
        # ================== 实体车核心修改 1 ==================
        # 必须强制设置为 False！使用真实的 Linux 系统时钟，否则节点会死锁。
        super().__init__('nav_controller_node', 
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, False)])
        # ====================================================
        
        # 初始化正规的 Nav2 动作客户端
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for Nav2 Action Server to become active (Real Robot Mode)...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 is active! Navigation microservice ready.')

        # 内部状态与记忆
        self.is_moving_enabled = False
        self.is_going_to_box = False 
        self.current_nav_intent = "IDLE" 
        self.is_navigating = False      
        self.current_goal_handle = None 
        
        self.spatial_memory = {
            'target_object': None,  
            'target_box': None     
        }

        # ================== 实体车核心修改 2 ==================
        # 探索边界限制：请根据你真实测试房间的大小修改这里！
        # 比如你的空地只有 2x2 米，那就设为 (-1.0, 1.0)
        self.map_limit_x = (-1.0, 1.0)
        self.map_limit_y = (-1.0, 1.0)
        # ====================================================

        # 话题发布与订阅
        self.pub_move_fb = self.create_publisher(Bool, '/move_feedback', 10)
        self.sub_moving = self.create_subscription(Bool, '/status/moving', self.moving_callback, 10)
        self.sub_to_box = self.create_subscription(Bool, '/status/going_to_box', self.going_to_box_callback, 10)
        self.sub_reset = self.create_subscription(Bool, '/status/reset_vision', self.reset_memory_callback, 10)
        self.sub_obj_pose = self.create_subscription(PoseStamped, '/target_object_pose', self.object_pose_callback, 10)
        self.sub_box_pose = self.create_subscription(PoseStamped, '/target_box_pose', self.box_pose_callback, 10)

        # 核心控制循环：每 0.5 秒检查一次大脑意图
        self.control_timer = self.create_timer(0.5, self.control_loop)

    # =========================================================================
    # 回调函数 (输入处理)
    # =========================================================================
    def reset_memory_callback(self, msg):
        if msg.data:
            self.get_logger().info('Clearing local navigation cache for new cycle.')
            self.spatial_memory['target_object'] = None
            self.spatial_memory['target_box'] = None

    def going_to_box_callback(self, msg):
        self.is_going_to_box = msg.data

    def moving_callback(self, msg):
        self.is_moving_enabled = msg.data
        if not self.is_moving_enabled:
            self.get_logger().info('Halt signal received. Canceling tasks.')
            if self.is_navigating and self.current_goal_handle:
                self.current_goal_handle.cancel_goal_async()
            self.current_nav_intent = "IDLE"
            self.is_navigating = False

    def object_pose_callback(self, msg):
        self.spatial_memory['target_object'] = msg
        if self.is_moving_enabled and not self.is_going_to_box and self.current_nav_intent == "EXPLORING":
            self.get_logger().info('Object found! Interrupting exploration.')
            if self.current_goal_handle:
                self.current_goal_handle.cancel_goal_async()
            self.current_nav_intent = "IDLE"
            self.is_navigating = False

    def box_pose_callback(self, msg):
        self.spatial_memory['target_box'] = msg
        if self.is_moving_enabled and self.is_going_to_box and self.current_nav_intent == "EXPLORING":
            self.get_logger().info('Box found! Interrupting exploration.')
            if self.current_goal_handle:
                self.current_goal_handle.cancel_goal_async()
            self.current_nav_intent = "IDLE"
            self.is_navigating = False

    # =========================================================================
    # 主控制引擎 (非阻塞状态机)
    # =========================================================================
    def control_loop(self):
        if not self.is_moving_enabled:
            return 

        if self.is_navigating:
            return

        target_key = 'target_box' if self.is_going_to_box else 'target_object'
        target_intent = "APPROACHING_BOX" if self.is_going_to_box else "APPROACHING_OBJ"
        
        # 1. 如果有明确坐标，前往目标
        if self.spatial_memory[target_key] is not None:
            self.send_nav_goal(self.spatial_memory[target_key], target_intent)
        
        # 2. 如果不知道坐标，进行随机探索
        else:
            self.get_logger().info(f'Target {target_key} unknown. Exploring arena...')
            goal_pose = self.generate_random_waypoint()
            self.send_nav_goal(goal_pose, "EXPLORING")

    # =========================================================================
    # Action Client 核心发送与回调逻辑
    # =========================================================================
    def generate_random_waypoint(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = random.uniform(self.map_limit_x[0], self.map_limit_x[1])
        goal.pose.position.y = random.uniform(self.map_limit_y[0], self.map_limit_y[1])
        yaw = random.uniform(-math.pi, math.pi)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def send_nav_goal(self, pose_stamped, intent):
        self.current_nav_intent = intent
        self.is_navigating = True
        
        # 刷新时间戳 (在实体车上，时间戳为 0 依然是最稳妥的做法，获取最新的 TF)
        pose_stamped.header.stamp.sec = 0
        pose_stamped.header.stamp.nanosec = 0
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        
        self.get_logger().info(f'[Action] Dispatching goal for: {intent}')
        
        # 异步发送目标
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2 server.')
            self.is_navigating = False
            return

        self.get_logger().info('Goal accepted. Navigating...')
        self.current_goal_handle = goal_handle
        
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        self.is_navigating = False
        self.current_goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Arrival Successful: {self.current_nav_intent}')
            # 只有在抓取或放置时，才给 FSM 发送到达反馈
            if self.current_nav_intent in ["APPROACHING_OBJ", "APPROACHING_BOX"]:
                msg = Bool()
                msg.data = True
                self.pub_move_fb.publish(msg)
            self.current_nav_intent = "IDLE" 
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Navigation task was canceled.')
        else:
            self.get_logger().error(f'Navigation failed with status code: {status}')
            self.current_nav_intent = "IDLE"

def main(args=None):
    rclpy.init(args=args)
    node = NavControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()