#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <cstdlib>
#include <ctime>
#include <cmath>

class RandomObs
{
public:
RandomObs()
    {
ROS_INFO("RandomObs 构造开始");
client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
if(!client_.waitForExistence(ros::Duration(2.0)))
        {
ROS_ERROR("等待 /gazebo/set_model_state 服务超时！");
return;
        }
ROS_INFO("服务连接成功");
srand((unsigned int)time(NULL));
// x固定在-1.4（小车正前方），y左右移动
target_x_ = -1.4;
target_y_ = getRand(-0.4, 0.4);
speed_ = 0.25;
ROS_INFO("初始化完成，初始目标 x:%.2f y:%.2f", target_x_, target_y_);
    }

double getRand(double min, double max)
    {
double r = (double)rand() / RAND_MAX;
return min + r * (max - min);
    }

void run()
    {
ros::Rate rate(20);
gazebo_msgs::ModelState state;
state.model_name = "dynamic_obs";
state.reference_frame = "world";

double cur_x = -1.4;
double cur_y = 0.0;
ROS_INFO("起点设置 x=-1.4 y=0.0");

while (ros::ok())
        {
double dx = target_x_ - cur_x;
double dy = target_y_ - cur_y;
double dist = sqrt(dx*dx + dy*dy);

if (dist < 0.15)
            {
target_x_ = -1.4;
target_y_ = getRand(-0.4, 0.4);

double sleep_t = getRand(0.0, 2.0);
ROS_INFO("到达目标，换新目标 x:%.2f y:%.2f，停顿 %.2fs", target_x_, target_y_, sleep_t);
ros::Duration(sleep_t).sleep();
continue;
            }

double vx = dx / dist * speed_;
double vy = dy / dist * speed_;

cur_x += vx * 0.05;
cur_y += vy * 0.05;

state.pose.position.x = cur_x;
state.pose.position.y = cur_y;
state.pose.position.z = 0.05;
state.pose.orientation.w = 1.0;

state.twist.linear.x = vx;
state.twist.linear.y = vy;
state.twist.linear.z = 0;

gazebo_msgs::SetModelState srv;
srv.request.model_state = state;
if (client_.call(srv))
            {
if (!srv.response.success)
                {
ROS_WARN("调用失败：%s", srv.response.status_message.c_str());
                }
            }
else
            {
ROS_ERROR("call service failed");
            }

rate.sleep();
        }
    }

private:
ros::NodeHandle nh_;
ros::ServiceClient client_;
double target_x_, target_y_;
double speed_;
};

int main(int argc, char **argv)
{
ros::init(argc, argv, "random_obs");
ROS_INFO("random_obs 节点启动成功！");
RandomObs obs;
obs.run();
return 0;
}
