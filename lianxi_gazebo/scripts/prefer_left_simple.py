#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate

class ForcePreferLeft:
    def __init__(self):
        self.cost_sub = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.cb)
        self.pub = rospy.Publisher("/move_base/local_costmap/costmap_updates", OccupancyGridUpdate, queue_size=5)
        self.bias = 45

    def cb(self, msg):
        info = msg.info
        w = info.width
        h = info.height
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        arr = np.array(msg.data, dtype=np.int8).reshape(h,w)

        for row in range(h):
            for col in range(w):
                wy = oy + row * res
                wx = ox + col * res
                # 小车前方区域，y>0（右侧）统一抬高代价
                if 0 < wx < 4.5 and wy > 0:
                    if 0 < arr[row,col] < 240:
                        arr[row,col] = min(254, arr[row,col] + self.bias)

        up = OccupancyGridUpdate()
        up.header = msg.header
        up.x = 0
        up.y = 0
        up.width = w
        up.height = h
        up.data = arr.flatten().tolist()
        self.pub.publish(up)

if __name__ == "__main__":
    rospy.init_node("prefer_left_simple")
    node = ForcePreferLeft()
    rospy.spin()
