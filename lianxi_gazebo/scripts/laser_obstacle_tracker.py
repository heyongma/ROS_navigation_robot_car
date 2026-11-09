#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

# ===================== 参数配置 =====================
CLUSTER_TOL = 0.3        # 欧式聚类距离阈值(m)
MIN_CLUSTER_SIZE = 3     # 一个簇最少多少个激光点，过滤噪点
MAX_TRACK_DIST = 0.6     # 前后帧匹配最大距离
VEL_THRESHOLD = 0.2      # 大于这个速度判定为动态障碍物
LOST_FRAME_MAX = 10      # 目标丢失多少帧后删除
MARKER_LIFETIME = 0.3

# 卡尔曼参数：状态 [x,y,vx,vy]
dt = 0.1
F = np.array([
    [1, 0, dt, 0],
    [0, 1, 0, dt],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])
H = np.array([[1,0,0,0],
              [0,1,0,0]])
Q = np.diag([0.01,0.01,0.05,0.05])
R = np.diag([0.04,0.04])
# ====================================================

class KalmanFilter:
    def __init__(self, x0, y0):
        self.x = np.array([[x0],[y0],[0.0],[0.0]])
        self.P = np.eye(4)*0.5
        self.lost_cnt = 0

    def predict(self):
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z):
        z = np.array(z).reshape(2,1)
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P

    def get_state(self):
        return self.x[0,0], self.x[1,0], self.x[2,0], self.x[3,0]

class LaserObstacleTracker:
    def __init__(self):
        rospy.init_node("laser_obstacle_tracker")
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=1)
        self.marker_pub = rospy.Publisher("/obstacle_markers", MarkerArray, queue_size=1)

        self.trackers = dict()  # key:id, value:KalmanFilter
        self.next_id = 0
        self.rate = rospy.Rate(10)

    def scan_to_points(self, scan):
        pts = []
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                pts.append([x,y])
            angle += scan.angle_increment
        return np.array(pts)

    def euclidean_cluster(self, points):
        clusters = []
        if len(points) < MIN_CLUSTER_SIZE:
            return clusters
        visited = np.zeros(len(points), dtype=bool)
        for i in range(len(points)):
            if not visited[i]:
                q = [i]
                visited[i] = True
                cluster = []
                while q:
                    idx = q.pop()
                    cluster.append(points[idx])
                    for n in range(len(points)):
                        if not visited[n]:
                            dist = np.hypot(points[idx][0]-points[n][0], points[idx][1]-points[n][1])
                            if dist < CLUSTER_TOL:
                                visited[n] = True
                                q.append(n)
                if len(cluster) >= MIN_CLUSTER_SIZE:
                    c = np.array(cluster)
                    cx = np.mean(c[:,0])
                    cy = np.mean(c[:,1])
                    clusters.append([cx, cy])
        return clusters

    def associate(self, detections):
        matched = {}
        track_ids = list(self.trackers.keys())
        for tid in track_ids:
            tx, ty, _, _ = self.trackers[tid].get_state()
            min_d = 999
            min_det = None
            for d in detections:
                dx, dy = d
                dis = np.hypot(tx-dx, ty-dy)
                if dis < min_d:
                    min_d = dis
                    min_det = d
            if min_det is not None and min_d < MAX_TRACK_DIST:
                matched[tid] = min_det
        return matched, detections

    def publish_marker(self):
        ma = MarkerArray()
        for tid, kf in self.trackers.items():
            x,y,vx,vy = kf.get_state()
            spd = np.hypot(vx, vy)
            m = Marker()
            m.header.frame_id = "laser_link"
            m.header.stamp = rospy.Time.now()
            m.ns = "obstacle"
            m.id = tid
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.2
            if spd > VEL_THRESHOLD:
                m.color.r = 1.0
                m.color.g = 0.0
            else:
                m.color.r = 0.0
                m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.8
            m.lifetime = rospy.Duration(MARKER_LIFETIME)
            ma.markers.append(m)
        self.marker_pub.publish(ma)

    def scan_cb(self, msg):
        pts = self.scan_to_points(msg)
        dets = self.euclidean_cluster(pts)
        matched, all_dets = self.associate(dets)

        # 更新匹配成功的跟踪器
        for tid, z in matched.items():
            kf = self.trackers[tid]
            kf.predict()
            kf.update(z)
            kf.lost_cnt = 0

        # 丢失目标计数
        for tid in list(self.trackers.keys()):
            if tid not in matched:
                kf = self.trackers[tid]
                kf.predict()
                kf.lost_cnt += 1
                if kf.lost_cnt > LOST_FRAME_MAX:
                    del self.trackers[tid]

        # 新建跟踪器
        used = list(matched.values())
        for d in all_dets:
            flag = False
            for u in used:
                if np.hypot(d[0]-u[0], d[1]-u[1]) < 0.05:
                    flag = True
                    break
            if not flag:
                self.trackers[self.next_id] = KalmanFilter(d[0], d[1])
                self.next_id += 1
        rospy.loginfo(f"检测到障碍物数量：{len(dets)} | 跟踪列表数量：{len(self.trackers)}")
        for tid, kf in self.trackers.items():
                x,y,vx,vy = kf.get_state()
                spd = np.hypot(vx, vy)
                rospy.loginfo(f"  id:{tid}, speed:{spd:.3f} m/s")

        self.publish_marker()

if __name__ == "__main__":
    try:
        t = LaserObstacleTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
