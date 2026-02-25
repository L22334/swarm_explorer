#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from enum import Enum
import math

# 定义业务状态机的状态
class State(Enum):
    IDLE = 1
    NAV_TO_SHELF = 2
    SCANNING = 3
    DOCKING = 4
    RETURN_HOME = 5

class WarehouseTaskManager(Node):
    def __init__(self):
        super().__init__('task_dispatcher_node')
        
        self.get_logger().info("智能仓储任务调度系统已启动...")
        self.state = State.IDLE
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.qr_sub = self.create_subscription(String, '/perception/qr_result', self.qr_callback, 10)
        
        self.get_logger().info("等待 Nav2 Action Server 上线...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 准备就绪！")

        self.start_task()

    def euler_to_quaternion(self, yaw):
        return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

    def create_pose(self, x, y, yaw):
        """创建目标位姿消息"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        
        q = self.euler_to_quaternion(yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def start_task(self):
        """阶段 1：派发任务，前往货架 1"""
        self.state = State.NAV_TO_SHELF
        self.get_logger().info("[状态变更] 接收到订单：前往 1 号货架取货！")
        
        shelf_pose = self.create_pose(x=2.5, y=0.9, yaw=1.5707) 
        self.send_nav_goal(shelf_pose)

    def send_nav_goal(self, pose):
        """发送导航目标到 Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.get_logger().info(f"正在发送导航目标: x={pose.pose.position.x}, y={pose.pose.position.y}")
        self._send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """确认 Nav2 是否接受了目标"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("导航目标被 Nav2 拒绝！")
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """导航完成后的回调处理"""
        result = future.result().result
        status = future.result().status
        
        if status == 4: # SUCCEEDED
            self.get_logger().info("成功到达目标点！")
            
            # 状态机流转
            if self.state == State.NAV_TO_SHELF:
                self.start_scanning()
            elif self.state == State.RETURN_HOME:
                self.state = State.IDLE
                self.get_logger().info("所有任务已圆满完成！小车已回到充电桩。")
                
        else:
            self.get_logger().error("导航失败，尝试重新规划或请求人工干预。")
            self.state = State.IDLE

    def start_scanning(self):
        """阶段 2：视觉对准与真实扫描"""
        self.state = State.SCANNING
        self.get_logger().info("[状态变更] 到达货架！已开启视觉监听，等待 C++ OpenCV 节点回传数据...")

    def qr_callback(self, msg):
        """处理 C++ 传回的真实二维码数据"""
        if self.state == State.SCANNING:
            if "Item_01" in msg.data:
                self.get_logger().info(f"[视觉融合] 真实识别成功！收到 C++ 传回数据: {msg.data}")
                self.start_docking()

    def start_docking(self):
        """阶段 3：精准对接"""
        self.state = State.DOCKING
        self.get_logger().info("[状态变更] 确认货物正确，执行精准对接 (Docking) 与模拟取货...")
        
        self.timer = self.create_timer(2.0, self.docking_complete)

    def docking_complete(self):
        """对接完成，准备返航"""
        self.timer.cancel()
        self.get_logger().info("取货完成！")
        
        self.state = State.RETURN_HOME
        self.get_logger().info("[状态变更] 正在返回充电桩...")
        
        home_pose = self.create_pose(x=0.0, y=0.0, yaw=3.14) 
        self.send_nav_goal(home_pose)

def main(args=None):
    rclpy.init(args=args)
    task_manager = WarehouseTaskManager()
    
    try:
        rclpy.spin(task_manager)
    except KeyboardInterrupt:
        task_manager.get_logger().info("系统被手动终止。")
    finally:
        task_manager.nav_client.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()