#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
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
struct TrajResult
{
    State end_state;
    bool collision;
};

// 轨迹仿真：逐步检查碰撞


class SimpleDWA
{
public:
    SimpleDWA()
    {
        nh_ = ros::NodeHandle("~");
        tf_listener_.reset(new tf::TransformListener());

        // 参数初始化
        params_.max_v = 0.75;
        params_.min_v = 0.0;
        params_.max_w = 1.2;
        params_.min_w = -1.0;

        params_.v_res = 0.05;
        params_.w_res = 0.05;

        params_.dt = 0.1;
        params_.sim_time = 1.4;
        params_.robot_radius = 0.12;

        params_.pdist_scale = 1.0;
        params_.gdist_scale = 1.0;
        params_.occdist_scale = 5.0;

        pub_cmd_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        sub_scan_ = nh_.subscribe("/scan", 10, &SimpleDWA::scanCb, this);
        sub_goal_ = nh_.subscribe("/move_base_simple/goal", 10, &SimpleDWA::goalCb, this);
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
      TrajResult simulateTraj(State s, double v, double w)
    {
        TrajResult res;
        res.collision = false;
        double t = 0.0;
        while (t < params_.sim_time)
        {
            if (checkSinglePoseCollision(s))
            {
                res.collision = true;
                break;
            }
            s.yaw += w * params_.dt;
            s.x += v * params_.dt * cos(s.yaw);
            s.y += v * params_.dt * sin(s.yaw);
            t += params_.dt;
        }
        res.end_state = s;
        return res;
    }

    bool checkSinglePoseCollision(const State& sim)
{
    if (!scan_.valid) return false;
    double dx_r = sim.x - state_.x;
    double dy_r = sim.y - state_.y;
    double sim_bx = dx_r * cos(-state_.yaw) - dy_r * sin(-state_.yaw);
    double sim_by = dx_r * sin(-state_.yaw) + dy_r * cos(-state_.yaw);

    for (int i = 0; i < scan_.ranges.size(); i++)
    {
        float r = scan_.ranges[i];
        if (r < scan_.range_min || r > scan_.range_max) continue;
        double ang = scan_.angle_min + i * scan_.angle_inc;
        double lx = r * cos(ang);
        double ly = r * sin(ang);
        double d = hypot(lx - sim_bx, ly - sim_by);
        if (d < params_.robot_radius)
        {
            return true;
        }
    }
    return false;
}


    // 障碍物代价
    // 修复后的障碍物代价
double calcOccCost(const State& sim)
{
    if (!scan_.valid) return 0.0;
    double min_dist = 1e9;
    double dx_r = sim.x - state_.x;
    double dy_r = sim.y - state_.y;
    double sim_bx = dx_r * cos(-state_.yaw) - dy_r * sin(-state_.yaw);
    double sim_by = dx_r * sin(-state_.yaw) + dy_r * cos(-state_.yaw);

    for (int i = 0; i < scan_.ranges.size(); i++)
    {
        float r = scan_.ranges[i];
        if (r < scan_.range_min || r > scan_.range_max) continue;
        double ang = scan_.angle_min + i * scan_.angle_inc;
        double lx = r * cos(ang);
        double ly = r * sin(ang);
        double d = hypot(lx - sim_bx, ly - sim_by);
        if (d < min_dist)
        {
            min_dist = d;
        }
    }
    return 1.0 / (min_dist + 1e-6);
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

            // ========== 每次循环查询 map -> base_link TF ==========
            tf::StampedTransform transform;
            try
            {
                tf_listener_->lookupTransform("map", "base_link", ros::Time(0), transform);
            }
            catch (tf::TransformException &ex)
            {
                ROS_WARN("Wait map->base_link tf: %s", ex.what());
                geometry_msgs::Twist zero;
                pub_cmd_.publish(zero);
                rate.sleep();
                continue;
            }
            // 从TF提取x,y,yaw
            state_.x = transform.getOrigin().x();
            state_.y = transform.getOrigin().y();
            tf::Quaternion q = transform.getRotation();
            tf::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, state_.yaw);

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
        if(!ros::ok()) return; //新增这一行，收到Ctrl+C直接跳出
        TrajResult traj_res = simulateTraj(state_, v, w);
        if (traj_res.collision) continue;
        State sim = traj_res.end_state;
        double occ_cost = calcOccCost(sim);

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
    ros::Subscriber sub_scan_;
    ros::Subscriber sub_goal_;

    boost::shared_ptr<tf::TransformListener> tf_listener_;

    Params params_;
    State state_;
    Goal goal_;
    ScanData scan_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_dwa_cpp");
    SimpleDWA node;
    ROS_INFO("TF version DWA started, listen map->base_link, output /cmd_vel");
    node.run();
    return 0;
}
