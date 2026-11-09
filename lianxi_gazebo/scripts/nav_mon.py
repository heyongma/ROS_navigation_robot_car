#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import csv
import os
import math
from time import time
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool

class NavMonitor:
    def __init__(self):
        rospy.init_node("nav_monitor_node", anonymous=False)
        # 参数：协方差阈值，amcl位置协方差[0]即x方向，超过判定定位丢失
        self.cov_threshold = rospy.get_param("~cov_threshold", 0.5)
        self.log_file = rospy.get_param("~log_file", "nav_log.csv")

        # 状态变量
        self.nav_state = "IDLE"  # IDLE / NAVIGATING / REACHED / PLAN_FAIL / LOST
        self.goal_start_time = None
        self.amcl_pose = None
        self.cmd_vel = Twist()
        self.trajectory = []
        self.collision_flag = False

        # 订阅
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.cb_move_base_status)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.cb_amcl_pose)
        rospy.Subscriber("/cmd_vel", Twist, self.cb_cmd_vel)
        rospy.Subscriber("/collision_event", Bool, self.cb_collision)

        # csv表头，如果文件不存在新建，增加collision_flag字段
        if not os.path.exists(self.log_file):
            with open(self.log_file, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp", "state", "x", "y", "yaw", "cov_x", "duration_s", "collision_flag"])

        rospy.loginfo("导航监控节点启动成功，日志文件：%s", self.log_file)
        rospy.spin()

    def cb_collision(self, msg):
        if msg.data:
            self.collision_flag = True

    def cb_cmd_vel(self, msg: Twist):
        self.cmd_vel = msg

    def cb_amcl_pose(self, msg: PoseWithCovarianceStamped):
        self.amcl_pose = msg
        # 提取x方向位置协方差，cov是36元素数组，0:x,7:y
        cov_x = msg.pose.covariance[0]
        cov_y = msg.pose.covariance[7]

        # 协方差超标：定位丢失
        if max(cov_x, cov_y) > self.cov_threshold:
            if self.nav_state != "LOST":
                self.nav_state = "LOST"
                rospy.logwarn("【告警】定位丢失！协方差 x:%.4f y:%.4f > threshold %.2f", cov_x, cov_y, self.cov_threshold)
            self.write_log(cov_x)
            return

        # 正常保存轨迹点
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.trajectory.append((rospy.Time.now().to_sec(), x, y))
        self.write_log(cov_x)

    def cb_move_base_status(self, msg: GoalStatusArray):
        if len(msg.status_list) == 0:
            self.nav_state = "IDLE"
            return
        status = msg.status_list[0].status

        if status == GoalStatus.ACTIVE:
            if self.nav_state not in ["NAVIGATING"]:
                self.nav_state = "NAVIGATING"
                self.goal_start_time = time()
                rospy.loginfo("【状态】开始导航")
        elif status == GoalStatus.SUCCEEDED:
            if self.nav_state != "REACHED":
                self.nav_state = "REACHED"
                dur = time() - self.goal_start_time if self.goal_start_time else 0
                rospy.loginfo("【完成】到达目标！导航耗时：%.2f s", dur)
        elif status in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
            if self.nav_state != "PLAN_FAIL":
                self.nav_state = "PLAN_FAIL"
                rospy.logwarn("【失败】规划失败/目标无法到达")

    def write_log(self, cov_x):
        if self.amcl_pose is None:
            return
        t = rospy.Time.now().to_sec()
        p = self.amcl_pose.pose.pose
        x = p.position.x
        y = p.position.y

        # 修复：标准四元数转yaw
        qx = p.orientation.x
        qy = p.orientation.y
        qz = p.orientation.z
        qw = p.orientation.w
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy*qy + qz*qz))

        dur = time() - self.goal_start_time if self.goal_start_time else 0

        # 碰撞优先标记COLLISION
        if self.collision_flag:
            state_out = "COLLISION"
            col_mark = "True"
            self.collision_flag = False
        else:
            state_out = self.nav_state
            col_mark = "False"

        with open(self.log_file, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "%.3f" % t,
                state_out,
                "%.4f" % x,
                "%.4f" % y,
                "%.4f" % yaw,
                "%.6f" % cov_x,
                "%.2f" % dur,
                col_mark
            ])

if __name__ == "__main__":
    try:
        NavMonitor()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点退出")
