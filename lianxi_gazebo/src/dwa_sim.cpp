#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <vector>
#include <algorithm>

struct Params
{
    double max_v;
    double min_v;
    double max_w;
    double min_w;

    double v_res;
    double w_res;

    double dt;
    double sim_time;
    double robot_radius;

    double pdist_scale;
    double gdist_scale;
    double occdist_scale;
};

struct State
{
    double x;
    double y;
    double yaw;
};

struct Goal
{
    double x;
    double y;
    bool valid;
    Goal() : x(0), y(0), valid(false) {}
};

struct ScanData
{
    std::vector<float> ranges;
    double angle_min;
    double angle_inc;
    double range_min;
    double range_max;
    bool valid;
    ScanData() : valid(false) {}
};

class SimpleDWA
{
public:
    SimpleDWA()
    {
        nh_ = ros::NodeHandle("~");

        // 参数初始化，和python版本一致
        params_.max_v = 0.5;
        params_.min_v = 0.0;
        params_.max_w = 1.0;
        params_.min_w = -1.0;

        params_.v_res = 0.05;
        params_.w_res = 0.05;

        params_.dt = 0.1;
        params_.sim_time = 2.0;
        params_.robot_radius = 0.12;

        params_.pdist_scale = 1.0;
        params_.gdist_scale = 1.0;
        params_.occdist_scale = 2.0;

        pub_cmd_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_dwa", 10);
        sub_odom_ = nh_.subscribe("/odom", 10, &SimpleDWA::odomCb, this);
        sub_scan_ = nh_.subscribe("/scan", 10, &SimpleDWA::scanCb, this);
        sub_goal_ = nh_.subscribe("/move_base_simple/goal", 10, &SimpleDWA::goalCb, this);
    }

    void odomCb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        state_.x = msg->pose.pose.position.x;
        state_.y = msg->pose.pose.position.y;
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, state_.yaw);
    }

    void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        scan_.ranges = msg->ranges;
        scan_.angle_min = msg->angle_min;
        scan_.angle_inc = msg->angle_increment;
        scan_.range_min = msg->range_min;
        scan_.range_max = msg->range_max;
        scan_.valid = true;
    }

    void goalCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        goal_.x = msg->pose.position.x;
        goal_.y = msg->pose.position.y;
        goal_.valid = true;
    }

    // 轨迹仿真：输入当前位姿 + v w，输出仿真终点
    State simulateTraj(State s, double v, double w)
    {
        double t = 0.0;
        while (t < params_.sim_time)
        s.yaw += w * params_.dt;
        s.x += v * params_.dt * cos(s.yaw);
        s.y += v * params_.dt * sin(s.yaw);
        t += params_.dt;
        return s;
    }

    // 障碍物代价
    double calcOccCost(const State& sim)
    {
        if (!scan_.valid) return 0.0;
        double min_r = 1e9;
        for (int i = 0; i < scan_.ranges.size(); i++)
        {
            float r = scan_.ranges[i];
            if (r < scan_.range_min || r > scan_.range_max) continue;
            double ang = scan_.angle_min + i * scan_.angle_inc;
            double lx = r * cos(ang);
            double ly = r * sin(ang);
            double d = hypot(lx, ly);
            if (d < min_r) min_r = d;
        }
        if (min_r < params_.robot_radius)
            return 1e9; // 碰撞，代价无穷大
        return 1.0 / (min_r + 1e-6);
    }

    // goal代价：距离+角度误差
    double calcGoalCost(const State& sim)
    {
        if (!goal_.valid) return 0.0;
        double dx = goal_.x - sim.x;
        double dy = goal_.y - sim.y;
        double dist = hypot(dx, dy);
        double ang_des = atan2(dy, dx);
        double ang_err = ang_des - sim.yaw;
        ang_err = atan2(sin(ang_err), cos(ang_err));
        return dist + fabs(ang_err);
    }

    // path代价：点到【当前机器人 -> 目标】线段距离，对应pdist
    double calcPathCost(const State& sim)
    {
        if (!goal_.valid) return 0.0;
        double rx = state_.x;
        double ry = state_.y;
        double dx = goal_.x - rx;
        double dy = goal_.y - ry;
        double t;
        if (fabs(dx) < 1e-6 && fabs(dy) < 1e-6)
            t = 0.0;
        else
            t = std::max(0.0, std::min(1.0, ((sim.x - rx)*dx + (sim.y - ry)*dy)/(dx*dx + dy*dy)));
        double proj_x = rx + t * dx;
        double proj_y = ry + t * dy;
        return hypot(sim.x - proj_x, sim.y - proj_y);
    }

    void run()
    {
        ros::Rate rate(10);
        while (ros::ok())
        {
            ros::spinOnce();
            if (!goal_.valid || !scan_.valid)
            {
                geometry_msgs::Twist zero;
                pub_cmd_.publish(zero);
                rate.sleep();
                continue;
            }

            double best_score = -1e9;
            double best_v = 0;
            double best_w = 0;

            // 速度采样遍历
            for (double v = params_.min_v; v <= params_.max_v + 1e-6; v += params_.v_res)
            {
                for (double w = params_.min_w; w <= params_.max_w + 1e-6; w += params_.w_res)
                {
                    State sim = simulateTraj(state_, v, w);
                    double occ_cost = calcOccCost(sim);
                    if (occ_cost > 1e8) continue;

                    double g_cost = calcGoalCost(sim);
                    double p_cost = calcPathCost(sim);

                    double total_cost = params_.pdist_scale * p_cost
                                      + params_.gdist_scale * g_cost
                                      + params_.occdist_scale * occ_cost;
                    double score = -total_cost;

                    if (score > best_score)
                    {
                        best_score = score;
                        best_v = v;
                        best_w = w;
                    }
                }
            }

            geometry_msgs::Twist cmd;
            cmd.linear.x = best_v;
            cmd.angular.z = best_w;
            pub_cmd_.publish(cmd);
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_cmd_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_scan_;
    ros::Subscriber sub_goal_;

    Params params_;
    State state_;
    Goal goal_;
    ScanData scan_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_dwa_cpp");
    SimpleDWA node;
    ROS_INFO("simple dwa cpp node started, output /cmd_vel_dwa");
    node.run();
    return 0;
}
