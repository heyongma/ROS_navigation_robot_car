#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
import math

class WaypointLoopNav:
    def __init__(self):
        rospy.init_node("way_point_many_node", anonymous=False)
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("等待连接 move_base ...")
        self.client.wait_for_server()
        rospy.loginfo("move_base 连接成功！")

        # 航点只保留 x y，不再带目标姿态
        self.waypoints = [
            [2.0,  0.0],
            [2.0, -2.0],
            [0.0, -2.0],
            [0.0,  0.0]
        ]
        self.wait_time = 2.0
        self.goal_timeout = 300.0
        self.xy_tolerance = 0.18   # 和yaml里面xy_goal_tolerance保持一致

        self.robot_x = 0.0
        self.robot_y = 0.0
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def send_goal(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)
        rospy.loginfo(f"下发目标 x={x}, y={y}")

    def run(self):
        while not rospy.is_shutdown():
            for idx, pt in enumerate(self.waypoints):
                target_x, target_y = pt
                self.send_goal(target_x, target_y)
                start_time = rospy.Time.now()
                arrived = False
                # 循环检测坐标
                while not rospy.is_shutdown():
                    dist = math.hypot(self.robot_x - target_x, self.robot_y - target_y)
                    if dist < self.xy_tolerance:
                        arrived = True
                        break
                    if (rospy.Time.now() - start_time).to_sec() > self.goal_timeout:
                        break
                    rospy.sleep(0.1)
                self.client.cancel_goal()
                if arrived:
                    rospy.loginfo(f"✅ 第{idx+1}号点位置到达，停留 {self.wait_time}s")
                    rospy.sleep(self.wait_time)
                else:
                    rospy.logwarn(f"❌ 第{idx+1}号点超时")
                    rospy.sleep(1.0)
            rospy.loginfo("==== 一圈结束，重新循环 ====")

if __name__ == "__main__":
    try:
        nav = WaypointLoopNav()
        nav.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点关闭")
