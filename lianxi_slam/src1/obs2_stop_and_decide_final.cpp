#include <ros/ros.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32MultiArray.h>
#include <cmath>
#include <vector>
#include <algorithm>

inline double normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

struct TriggerWindow
{
    double rx_min;
    double rx_max;
    int obs_id;
};

class ObsSampleDecideNode
{
public:
    ObsSampleDecideNode(ros::NodeHandle& nh)
    {
        nh.param<double>("lookahead", lookahead_, 0.8);
        nh.param<double>("max_vel", max_vel_, 0.3);
        nh.param<double>("max_omega_out", max_omega_out_, 0.6);
        nh.param<double>("detour_offset", detour_offset_, 0.6);
        nh.param<double>("step", step_, 0.15);
        nh.param<int>("sample_total", sample_total_, 11);
        nh.param<double>("sample_timeout", sample_timeout_, 3.0); // 采样超时保护

        windows_ = {
            {0.1,   1.2,    0},
            {3.5,   4.6,    1},
            {-4.5, -3.4,    2}
        };

        side_pub_ = nh.advertise<std_msgs::Int32MultiArray>("/detour_side_array", 10);
        cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        path_pub_ = nh.advertise<nav_msgs::Path>("/show_path", 10);

        obs_sub_ = nh.subscribe("/move_base/TebLocalPlannerROS/obstacles", 10, &ObsSampleDecideNode::obsCallback, this);
        pose_sub_ = nh.subscribe("/amcl_pose",10, &ObsSampleDecideNode::poseCallback, this);
        odom_sub_ = nh.subscribe("/odom",10, &ObsSampleDecideNode::odomCallback, this);
        goal_sub_ = nh.subscribe("/move_base_simple/goal",10, &ObsSampleDecideNode::goalCallback, this);

        ROS_INFO("obs_sample_decide_node 启动完成；rx窗口判定触发；窗口内强制停车");
    }

    void run()
    {
        ros::Rate rate(20);
        while (ros::ok())
        {
            ros::spinOnce();
            geometry_msgs::Twist cmd{};

            // 采样超时保护：采样中超过设定时间还没完成采样，强制重置
            if(sampling_)
            {
                double elapsed = (ros::Time::now() - sample_start_t_).toSec();
                if(elapsed > sample_timeout_)
                {
                    ROS_ERROR("[SAMPLING_TIMEOUT] 采样超时 %.2f s，重置采样状态！active_obs_id=%d", elapsed, active_obs_id_);
                    sampling_ = false;
                    state_ = IDLE;
                    sample_count_ = 0;
                    left_cnt_ = 0;
                    right_cnt_ = 0;
                }
            }

            if (in_trigger_zone_)
            {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                cmd_pub_.publish(cmd);
                ROS_INFO_THROTTLE(0.5, "[FORCE_STOP] 在触发窗口内，active_obs_id=%d，rx=%.3f ry=%.3f", active_obs_id_, rx, ry);
                rate.sleep();
                continue;
            }

            if (!robot_valid_)
            {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                ROS_INFO_THROTTLE(1.0, "[NO_POSE] 未收到amcl_pose，定位无效");
            }
            else if (!have_goal_)
            {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                ROS_INFO_THROTTLE(1.0, "[NO_GOAL] 没有收到导航目标点");
            }
            else
            {
                ROS_INFO_THROTTLE(1.0, "[RUNNING] 定位有效，存在目标，state=%d, final_side=%d", (int)state_, final_side_);
                nav_msgs::Path path = buildPath();
                path_pub_.publish(path);
                purePursuit(path, cmd);
            }
            cmd_pub_.publish(cmd);
            rate.sleep();
        }
    }

private:
    double lookahead_;
    double max_vel_;
    double max_omega_out_;
    double detour_offset_;
    double step_;
    int sample_total_;
    double sample_timeout_;

    std::vector<TriggerWindow> windows_;

    double rx_{0.0}, ry_{0.0};
    double odom_yaw_{0.0};
    bool robot_valid_{false};
    bool have_goal_{false};
    double gx_{0.0}, gy_{0.0};

    enum State
    {
        IDLE,
        DECIDED
    };
    State state_{IDLE};
    int final_side_{0};
    int active_obs_id_{-1};

    ros::Time sample_start_t_;
    int sample_count_{0};
    int left_cnt_{0};
    int right_cnt_{0};
    bool sampling_{false};
    bool in_trigger_zone_{false};

    ros::Publisher side_pub_;
    ros::Publisher cmd_pub_;
    ros::Publisher path_pub_;
    ros::Subscriber obs_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber goal_sub_;

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        rx = msg->pose.pose.position.x;
        ry = msg->pose.pose.position.y;
        robot_valid_ = true;

        const auto& q = msg->pose.pose.orientation;
        double qw = q.w, qz = q.z, qx = q.x, qy = q.y;

        bool old_zone = in_trigger_zone_;
        in_trigger_zone_ = false;
        active_obs_id_ = -1;

        for (const auto& win : windows_)
        {
            if (rx >= win.rx_min && rx <= win.rx_max)
            {
                in_trigger_zone_ = true;
                active_obs_id_ = win.obs_id;
                break;
            }
        }

        if (old_zone != in_trigger_zone_)
        {
            if(in_trigger_zone_)
            {
                ROS_INFO("[ZONE_ENTER] rx=%.3f ry=%.3f，进入触发窗口，绑定obs_id=%d", rx, ry, active_obs_id_);
            }
            else
            {
                ROS_INFO("[ZONE_LEAVE] rx=%.3f ry=%.3f，离开触发窗口", rx, ry);
            }
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        const auto& q = msg->pose.pose.orientation;
        double qw = q.w, qz = q.z, qx = q.x, qy = q.y;
        odom_yaw_ = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qz * qz + qy * qy));
        ROS_INFO_THROTTLE(2.0, "[ODOM_YAW] yaw=%.3f rad", odom_yaw_);
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        gx = msg->pose.position.x;
        gy = msg->pose.position.y;
        have_goal_ = true;
        final_side_ = 0;
        state_ = IDLE;
        sampling_ = false;
        sample_count_ = 0;
        left_cnt_ = 0;
        right_cnt_ = 0;
        in_trigger_zone_ = false;
        active_obs_id_ = -1;
        ROS_INFO("[RESET_GOAL] 收到新目标 (%.2f, %.2f)，所有状态重置", gx, gy);
    }

    void obsCallback(const costmap_converter::ObstacleArrayMsg::ConstPtr& msg)
    {
        if (!robot_valid_)
        {
            ROS_INFO_THROTTLE(1.0, "[OBS_SKIP] 定位无效，跳过障碍物消息");
            return;
        }
        if (state_ == DECIDED)
        {
            ROS_INFO_THROTTLE(1.0, "[OBS_SKIP] 已经完成绕行决策，不再采样");
            return;
        }
        if (!in_trigger_zone_ || active_obs_id_ < 0)
        {
            ROS_INFO_THROTTLE(1.0, "[OBS_SKIP] 不在触发窗口或者无绑定obs_id");
            return;
        }

        double ox = 0, oy = 0;
        bool obs_ok = false;
        for (const auto& obs : msg->obstacles)
        {
            if (obs.id == active_obs_id_)
            {
                ox = obs.polygon.points[0].x;
                oy = obs.polygon.points[0].y;
                obs_ok = true;
                break;
            }
        }
        if (!obs_ok)
        {
            ROS_WARN("[OBS_MISS] 当前消息里找不到id=%d的障碍物", active_obs_id_);
            return;
        }

        if (!sampling_)
        {
            sampling_ = true;
            sample_start_t_ = ros::Time::now();
            sample_count_ = 0;
            left_cnt_ = 0;
            right_cnt_ = 0;
            ROS_INFO("[SAMPLING_START] obs_id=%d, car rx=%.3f ry=%.3f, obs (%.3f,%.3f)", active_obs_id_, rx, ry, ox, oy);
        }

        double dx_w = ox - rx;
        double dy_w = oy - ry;
        double c = cos(odom_yaw_);
        double s = sin(odom_yaw_);
        double local_y = -dx_w * s + dy_w * c;

        double elapsed = (ros::Time::now() - sample_start_t_).toSec();
        int new_sample = static_cast<int>(elapsed);
        if (new_sample > sample_count_ && sample_count_ < sample_total_)
        {
            sample_count_ = new_sample;
            if (local_y > 0)
            {
                left_cnt_++;
                ROS_INFO("[SAMPLING_STEP] sample=%d, local_y=%.3f -> LEFT, left_cnt=%d", sample_count_, local_y, left_cnt_);
            }
            else
            {
                right_cnt_++;
                ROS_INFO("[SAMPLING_STEP] sample=%d, local_y=%.3f -> RIGHT, right_cnt=%d", sample_count_, local_y, right_cnt_);
            }
        }

        if (sample_count_ >= sample_total_)
        {
            if (left_cnt_ > right_cnt_)
            {
                final_side_ = 1;
                ROS_INFO("[DECIDE_RESULT] left_cnt:%d > right_cnt:%d → 选择向右绕行 final_side=1", left_cnt_, right_cnt_);
            }
            else
            {
                final_side_ = -1;
                ROS_INFO("[DECIDE_RESULT] left_cnt:%d <= right_cnt:%d → 选择向左绕行 final_side=-1", left_cnt_, right_cnt_);
            }
            state_ = DECIDED;
            sampling_ = false;
        }
    }

    nav_msgs::Path buildPath()
    {
        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();

        double dx_goal = gx - rx;
        double dy_goal = gy - ry;
        double dist_goal = hypot(dx_goal, dy_goal);
        int point_num = static_cast<int>(dist_goal / step_) + 2;
        ROS_INFO_THROTTLE(1.0, "[BUILD_PATH] 目标距离=%.3f m，路径点数=%d，final_side=%d", dist_goal, point_num, final_side_);

        double c = cos(odom_yaw_);
        double s = sin(odom_yaw_);

        for (int i = 0; i <= point_num; ++i)
        {
            double lx = i * step_;
            double ratio = std::min(lx / 1.0, 1.0);
            double ly = final_side_ * detour_offset_ * ratio;

            double wx = rx + lx * c - ly * s;
            double wy = ry + lx * s + ly * c;

            geometry_msgs::PoseStamped p;
            p.header.frame_id = "map";
            p.header.stamp = ros::Time::now();
            p.pose.position.x = wx;
            p.pose.position.y = wy;
            p.pose.orientation.w = 1.0;
            path.poses.push_back(p);
        }
        return path;
    }

    void purePursuit(const nav_msgs::Path& path, geometry_msgs::Twist& cmd)
    {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        if (path.poses.empty())
        {
            ROS_WARN("[PURE_PURSUIT] path为空，输出零速");
            return;
        }

        int target_idx = 0;
        double min_err = 1e9;
        for (int i = 0; i < path.poses.size(); ++i)
        {
            double dx = path.poses[i].pose.position.x - rx;
            double dy = path.poses[i].pose.position.y - ry;
            double d = hypot(dx, dy);
            double err = fabs(d - lookahead_);
            if (err < min_err)
            {
                min_err = err;
                target_idx = i;
            }
        }

        double tx = path.poses[target_idx].pose.position.x;
        double ty = path.poses[target_idx].pose.position.y;
        double dx = tx - rx;
        double dy = ty - ry;
        double dist = hypot(dx, dy);
        if (dist < 1e-6)
        {
            ROS_WARN("[PURE_PURSUIT] 跟踪点离小车太近，停止输出速度");
            return;
        }

        double alpha = normalizeAngle(atan2(dy, dx) - odom_yaw_);
        double omega = 2 * max_vel_ * sin(alpha) / dist;
        omega = std::clamp(omega, -max_omega_out_, max_omega_out_);

        cmd.linear.x = max_vel_;
        cmd.angular.z = omega;
        ROS_INFO_THROTTLE(1.0, "[PURE_PURSUIT] target(%.2f,%.2f), dist=%.3f, alpha=%.3f, omega=%.3f, v=%.3f",
            tx, ty, dist, alpha, omega, cmd.linear.x);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obs_sample_decide_node");
    ros::NodeHandle nh("~");
    ObsSampleDecideNode node(nh);
    node.run();
    return 0;
}
