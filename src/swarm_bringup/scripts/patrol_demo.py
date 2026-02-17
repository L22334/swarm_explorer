#!/usr/bin/env python3
import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    # 1. 初始化 ROS 2 节点
    rclpy.init()
    
    # 2. 创建导航指挥官 (Navigator)
    navigator = BasicNavigator()

    # 3. 等待 Nav2 系统完全启动
    # 这一步很重要，它会检查 AMCL, Planner, Controller 是否都准备好了
    print("正在等待 Nav2 启动...")
    navigator.waitUntilNav2Active()
    print("Nav2 已就绪！开始巡逻任务！")

    # === 定义巡逻点坐标 (基于你刚才的测量) ===
    # 格式: [x, y, z, orientation_w]
    # orientation_w = 1.0 代表车头朝向默认方向(东)。
    # 如果你想让车头转动，需要调整四元数 (这里暂时简化处理)
    
    goal_poses = []
    
    # --- 货架 A ---
    shelf_a = PoseStamped()
    shelf_a.header.frame_id = 'map'
    shelf_a.header.stamp = navigator.get_clock().now().to_msg()
    shelf_a.pose.position.x = 2.05
    shelf_a.pose.position.y = 1.20
    shelf_a.pose.orientation.w = 1.0 # 车头朝东
    goal_poses.append(shelf_a)

    # --- 货架 B ---
    shelf_b = PoseStamped()
    shelf_b.header.frame_id = 'map'
    shelf_b.header.stamp = navigator.get_clock().now().to_msg()
    shelf_b.pose.position.x = 2.02
    shelf_b.pose.position.y = -0.98
    shelf_b.pose.orientation.w = 1.0
    goal_poses.append(shelf_b)

    # --- 回到起点 ---
    start_point = PoseStamped()
    start_point.header.frame_id = 'map'
    start_point.header.stamp = navigator.get_clock().now().to_msg()
    start_point.pose.position.x = 0.0
    start_point.pose.position.y = 0.0
    start_point.pose.orientation.w = 1.0
    goal_poses.append(start_point)

    # === 开始循环巡逻 ===
    # 我们让它跑 2 圈
    for lap in range(2):
        print(f"--- 开始第 {lap + 1} 圈巡逻 ---")
        
        for i, goal in enumerate(goal_poses):
            print(f"正在前往第 {i+1} 个目标点: (x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f})...")
            
            # --- 核心指令: 去这里！ ---
            navigator.goToPose(goal)

            # --- 循环检查任务状态 ---
            while not navigator.isTaskComplete():
                # 这里的 feedback 可以获取剩余距离等信息
                feedback = navigator.getFeedback()
                # print(f"距离目标还有: {feedback.distance_remaining:.2f} 米")
                # 稍微睡一下，避免 CPU 占用过高
                time.sleep(0.5)

            # --- 检查最终结果 ---
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print("✅ 到达目标！")
                # 到了之后停 2 秒，模拟“正在搬运货物”
                time.sleep(2.0)
            elif result == TaskResult.CANCELED:
                print("⛔ 任务被取消")
                exit(1)
            elif result == TaskResult.FAILED:
                print("❌ 任务失败 (可能是路被堵住了)")
                # 即使失败也继续去下一个点
    
    print("🎉 所有巡逻任务完成！机器人准备下班！")
    
    # 4. 关闭节点
    rclpy.shutdown()

if __name__ == '__main__':
    main()