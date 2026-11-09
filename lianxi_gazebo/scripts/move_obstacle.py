#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseStamped, TwistStamped
import random

if __name__ == '__main__':
    rospy.init_node('markov_obstacle_node')
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    obs_pub = rospy.Publisher('/dynamic_obstacle_info', PoseStamped, queue_size=10)
    obs_twist_pub = rospy.Publisher('/dynamic_obstacle_twist', TwistStamped, queue_size=10)
    rate = rospy.Rate(20)

    x0 = 2.0
    y0 = -1.0
    range_x = 1.0
    range_y = 0.6

    X_MIN = x0 - range_x
    X_MAX = x0 + range_x
    Y_MIN = y0 - range_y
    Y_MAX = y0 + range_y

    x = x0
    y = y0

    state = 0
    vx = 0.01
    vy = 0.008

    trans_prob = [
            [0.40, 0.15, 0.15, 0.30],
            [0.15, 0.40, 0.15, 0.30],
            [0.15, 0.15, 0.40, 0.30],
            [0.20, 0.15, 0.15, 0.50]
        ]

    switch_interval = 18
    count = 0

    rospy.loginfo("动态障碍物节点启动")

    while not rospy.is_shutdown():
        msg = ModelState()
        msg.model_name = "moving_obstacle"

        if count >= switch_interval:
            r = random.random()
            sum_p = 0.0
            for next_s in range(4):
                sum_p += trans_prob[state][next_s]
                if r < sum_p:
                    state = next_s
                    break
            count = 0

        if state == 0:
            vx = 0.01
            vy = 0.0
        elif state == 1:
            vx = 0.006
            vy = 0.008
        elif state == 2:
            vx = 0.006
            vy = -0.008
        elif state == 3:
            vx = 0.0
            vy = 0.0

        x += vx
        y += vy

        if x <= X_MIN or x >= X_MAX:
            vx = -vx
            x = max(X_MIN, min(X_MAX, x))
        if y <= Y_MIN or y >= Y_MAX:
            vy = -vy
            y = max(Y_MIN, min(Y_MAX, y))

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.05
        msg.pose.orientation.w = 1.0
        pub.publish(msg)

        # 发布位置
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        obs_pub.publish(pose_msg)

        # 发布速度
        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.header.frame_id = "map"
        twist_msg.twist.linear.x = vx
        twist_msg.twist.linear.y = vy
        obs_twist_pub.publish(twist_msg)

        count += 1
        rate.sleep()
