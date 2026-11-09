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

ros::Publisher side_pub;
ros::Publisher cmd_pub;
ros::Publisher path_pub;
ros::Subscriber obs_sub;
ros::Subscriber pose_sub;
ros::Subscriber odom_sub;
ros::Subscriber goal_sub;

const double LOOKAHEAD = 0.8;
const double MAX_VEL = 0.3;
const double MAX_OMEGA_OUT = 0.6;
const double DETOUR_OFFSET = 0.6;
const double STEP = 0.15;

//=========================================================================
// 【 !!! 禁止修改下面WINDOWS数组内所有数值 !!! 】
// 窗口为小车map坐标系 X(rx) 触发区间，不是Y(ry)！此前错误误用ry判断已纠正
// id0：rx 0.1 ~ 1.2
// id1：rx 3.5 ~ 4.6
// id2：rx -4.5 ~ -3.4
//=========================================================================
// --------------------------仿真环境信息【注释，仅记录，不参与运算】------------------------
// 小车出生信息（launch文件）
// 模型名：car_to
// 出生map坐标：x=0.0, y=0.0, z=0.05；Yaw=3.1416(π)，车头朝向Y轴负方向；小车直行时y不断减小，rx发生变化用于触发窗口
//
// 三个移动障碍物信息（obs_auto.cpp）
// id=0，模型名dynamic_obs，固定X=-1.4，仅Y轴正弦往复运动，y∈[-0.5, 0.5]
// id=1，模型名dynamic_obs_1，固定X=-6.0，仅Y轴正弦往复运动，y∈[-0.5, 0.5]
// id=2，模型名dynamic_obs_2，固定X=2.0， 仅Y轴正弦往复运动，y∈[-0.5, 0.5]
// 障碍物话题：/move_base/TebLocalPlannerROS/obstacles
// ---------------------------------------------------------------------------------------
struct TriggerWindow
{
    double rx_min;
    double rx_max;
    int obs_id;
};
const TriggerWindow WINDOWS[3] =
{
    {0.1,   1.2,    0},
    {3.5,   4.6,    1},
    {-4.5, -3.4,    2}
};
//=========================================================================

double rx = 0.0, ry = 0.0;
double odom_yaw = 0.0;
bool robot_valid = false;
bool have_goal = false;
double gx = 0.0, gy = 0.0;

enum State
{
    IDLE,
    DECIDED
};
State state = IDLE;
int final_side = 0;
int active_obs_id = -1;   // 当前命中窗口对应的障碍物id

ros::Time sample_start_t;
int sample_count = 0;
int left_cnt = 0;
int right_cnt = 0;
const int SAMPLE_TOTAL = 11;
bool sampling = false;
bool in_trigger_zone = false;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    rx = msg->pose.pose.position.x;
    ry = msg->pose.pose.position.y;
    double qw = msg->pose.pose.orientation.w;
    double qz = msg->pose.pose.orientation.z;
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    robot_valid = true;

    bool old = in_trigger_zone;
    in_trigger_zone = false;
    active_obs_id = -1;

    // 使用小车rx(X坐标)判断触发窗口，已纠正之前ry的错误
    for(int i = 0; i < 3; ++i)
    {
        if(rx >= WINDOWS[i].rx_min && rx <= WINDOWS[i].rx_max)
        {
            in_trigger_zone = true;
            active_obs_id = WINDOWS[i].obs_id;
            break;
        }
    }

    if(old != in_trigger_zone)
    {
        ROS_INFO("[ZONE_CHANGE] rx=%.3f ry=%.3f in_trigger_zone=%s active_obs_id=%d",
                 rx, ry, in_trigger_zone?"true":"false", active_obs_id);
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double qw = msg->pose.pose.orientation.w;
    double qz = msg->pose.pose.orientation.z;
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    odom_yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qz*qz + qy*qy));
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    gx = msg->pose.position.x;
    gy = msg->pose.position.y;
    have_goal = true;
    ROS_INFO("[RESET] goal (%.2f,%.2f)", gx, gy);
    final_side = 0;
    state = IDLE;
    sampling = false;
    sample_count = 0;
    left_cnt = 0;
    right_cnt = 0;
    in_trigger_zone = false;
    active_obs_id = -1;
}

void obsCallback(const costmap_converter::ObstacleArrayMsg::ConstPtr& msg)
{
    if (!robot_valid || state == DECIDED)
        return;
    if(!in_trigger_zone || active_obs_id < 0)
        return;

    double ox = 0, oy = 0;
    bool obs_ok = false;
    for(auto &obs : msg->obstacles)
    {
        if(obs.id == active_obs_id)
        {
            ox = obs.polygon.points[0].x;
            oy = obs.polygon.points[0].y;
            obs_ok = true;
            break;
        }
    }
    if(!obs_ok)
        return;

    if(in_trigger_zone && !sampling)
    {
        sampling = true;
        sample_start_t = ros::Time::now();
        sample_count = 0;
        left_cnt = 0;
        right_cnt = 0;
        // 新增：进入采样瞬间，打印时间、当前小车自身rx ry坐标、对应障碍物id
        ROS_INFO("[SAMPLING_START] enter_sampling! time=%.2f, active_obs_id=%d, car_pos: rx=%.3f ry=%.3f",
                 ros::Time::now().toSec(), active_obs_id, rx, ry);
    }
    if(!sampling)
        return;

    double dx_w = ox - rx;
    double dy_w = oy - ry;
    double c = cos(odom_yaw);
    double s = sin(odom_yaw);
    double local_y = -dx_w * s + dy_w * c;

    double elapsed = (ros::Time::now() - sample_start_t).toSec();
    int new_sample = static_cast<int>(elapsed);
    if(new_sample > sample_count && sample_count < SAMPLE_TOTAL)
    {
        sample_count = new_sample;
        if(local_y > 0)
        {
            left_cnt++;
            ROS_INFO("[SAMPLE %d] left %d, car_now rx=%.3f ry=%.3f", sample_count, left_cnt, rx, ry);
        }
        else
        {
            right_cnt++;
            ROS_INFO("[SAMPLE %d] right %d, car_now rx=%.3f ry=%.3f", sample_count, right_cnt, rx, ry);
        }
    }

    if(sample_count >= SAMPLE_TOTAL)
    {
        if(left_cnt > right_cnt)
        {
            final_side = 1;
            ROS_INFO("[DECIDE] go right, final_side=1");
        }
        else
        {
            final_side = -1;
            ROS_INFO("[DECIDE] go left, final_side=-1");
        }
        state = DECIDED;
        sampling = false;
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
    int point_num = static_cast<int>(dist_goal / STEP) + 2;

    double c = cos(odom_yaw);
    double s = sin(odom_yaw);

    for(int i=0; i<=point_num; i++)
    {
        double lx = i * STEP;
        double ratio = std::min(lx / 1.0, 1.0);
        double ly = final_side * DETOUR_OFFSET * ratio;

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

void purePursuit(const nav_msgs::Path& path, geometry_msgs::Twist &cmd)
{
    if(path.poses.empty())
    {
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        return;
    }

    int target_idx = 0;
    double min_err = 1e9;
    for(int i=0; i<path.poses.size(); i++)
    {
        double dx = path.poses[i].pose.position.x - rx;
        double dy = path.poses[i].pose.position.y - ry;
        double d = hypot(dx, dy);
        if(fabs(d - LOOKAHEAD) < min_err)
        {
            min_err = fabs(d - LOOKAHEAD);
            target_idx = i;
        }
    }
    double tx = path.poses[target_idx].pose.position.x;
    double ty = path.poses[target_idx].pose.position.y;

    double dx = tx - rx;
    double dy = ty - ry;
    double alpha = atan2(dy, dx) - odom_yaw;
    while(alpha > M_PI) alpha -= 2*M_PI;
    while(alpha < -M_PI) alpha += 2*M_PI;

    double dist = hypot(dx, dy);
    double omega = 2 * MAX_VEL * sin(alpha) / dist;
    if(omega > MAX_OMEGA_OUT) omega = MAX_OMEGA_OUT;
    if(omega < -MAX_OMEGA_OUT) omega = -MAX_OMEGA_OUT;

    cmd.linear.x = MAX_VEL;
    cmd.angular.z = omega;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obs_sample_decide_node");
    ros::NodeHandle nh;

    side_pub = nh.advertise<std_msgs::Int32MultiArray>("/detour_side_array", 10);
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    path_pub = nh.advertise<nav_msgs::Path>("/show_path",10);

    obs_sub = nh.subscribe("/move_base/TebLocalPlannerROS/obstacles", 10, obsCallback);
    pose_sub = nh.subscribe("/amcl_pose",10,poseCallback);
    odom_sub = nh.subscribe("/odom",10,odomCallback);
    goal_sub = nh.subscribe("/move_base_simple/goal",10,goalCallback);

    ROS_INFO("启动：使用rx(X坐标)判定触发窗口；进窗口立刻强制停车；仅匹配对应id障碍物才采样");
    ros::Rate rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;

        // 最高优先级强制停车，判断小车rx命中三组窗口之一，不依赖障碍物消息
        if(in_trigger_zone)
        {
            cmd_pub.publish(stop_cmd);
            ROS_INFO_THROTTLE(0.5, "[FORCE_STOP] 强制零速下发中, active_obs_id=%d", active_obs_id);
            rate.sleep();
            continue;
        }

        if(robot_valid && have_goal)
        {
            nav_msgs::Path path = buildPath();
            path_pub.publish(path);
            geometry_msgs::Twist cmd;
            purePursuit(path, cmd);
            cmd_pub.publish(cmd);
        }
        rate.sleep();
    }
    return 0;
}
