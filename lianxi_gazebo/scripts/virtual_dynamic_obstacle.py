#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import PointCloud2, PointField
import struct

class DynamicObstacleInjector:
    def __init__(self):
        self.sub_pose = rospy.Subscriber("/dynamic_obstacle_info", PoseStamped, self.pose_cb)
        self.sub_twist = rospy.Subscriber("/dynamic_obstacle_twist", TwistStamped, self.twist_cb)
        self.pub_cloud = rospy.Publisher("/move_base/local_costmap/dynamic_obs_cloud", PointCloud2, queue_size=10)
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.dt_predict = 0.3
        self.rate = rospy.Rate(20)

    def pose_cb(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

    def twist_cb(self, msg):
        self.vx = msg.twist.linear.x
        self.vy = msg.twist.linear.y

    def publish_cloud(self, x, y, frame_id):
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = rospy.Time.now()
        cloud_msg.header.frame_id = frame_id
        cloud_msg.height = 1
        cloud_msg.width = 1
        cloud_msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        data = struct.pack('fff', x, y, 0.05)
        cloud_msg.data = data
        self.pub_cloud.publish(cloud_msg)

    def run(self):
        while not rospy.is_shutdown():
            pred_x = self.x + self.vx * self.dt_predict
            pred_y = self.y + self.vy * self.dt_predict
            self.publish_cloud(pred_x, pred_y, "map")
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("virtual_dynamic_obstacle_injector")
    node = DynamicObstacleInjector()
    node.run()
