#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dtet.py
激光雷达碰撞检测：雷达测距小于阈值判定碰撞
发布 /collision_event Bool=True 对接nav_monitor
"""
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class CollisionDetector:
    def __init__(self):
        rospy.init_node("collision_detector_node", anonymous=False)
        # 碰撞距离阈值，单位米，可以按需调小
        self.collision_dist = 0.14
        self.pub_collision = rospy.Publisher("/collision_event", Bool, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.callback)
        rospy.loginfo("激光碰撞检测节点启动，碰撞阈值：%.2f m", self.collision_dist)

    def callback(self, msg):
        # 过滤掉无效值，找最小距离
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if not valid_ranges:
            return
        min_r = min(valid_ranges)
        if min_r < self.collision_dist:
            self.pub_collision.publish(True)
            rospy.logwarn("[COLLISION] 激光距离过近！当前最小距离：%.3f m", min_r)

if __name__ == "__main__":
    try:
        node = CollisionDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点退出")
