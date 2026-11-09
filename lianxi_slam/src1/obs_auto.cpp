#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <cmath>
#include <string>

int main(int argc, char** argv)
{
ros::init(argc, argv, "obs_auto");
ros::NodeHandle nh;

ros::ServiceClient set_model_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
ros::Publisher obs_pub = nh.advertise<costmap_converter::ObstacleArrayMsg>("/move_base/TebLocalPlannerROS/obstacles", 10);

set_model_state_client.waitForExistence(ros::Duration(5.0));
ROS_INFO("gazebo set_model_state service ready");

std::vector<double> offset_x = {-1.4, -6.0, 2.0};
std::vector<double> amp = {0.4, 0.4, 0.4};
std::vector<double> omega = {0.4, 0.4, 0.4};
std::vector<double> phase = {0, 1.57, 3.14};
std::vector<std::string> model_names = {"dynamic_obs", "dynamic_obs_1", "dynamic_obs_2"};

ros::Rate rate(20);
double t = 0.0;

while(ros::ok())
{
t += 0.05;
costmap_converter::ObstacleArrayMsg obs_array;
obs_array.header.stamp = ros::Time::now();
obs_array.header.frame_id = "map";

for(int i=0; i<3; i++)
{
double y_raw = amp[i] * sin(omega[i] * t + phase[i]);
double y = y_raw;
if(y > 0.5) y = 0.5;
if(y < -0.5) y = -0.5;

gazebo_msgs::ModelState model_state;
model_state.model_name = model_names[i];
model_state.pose.position.x = offset_x[i];
model_state.pose.position.y = y;
model_state.pose.position.z = 0.27;
model_state.pose.orientation.w = 1.0;

gazebo_msgs::SetModelState srv;
srv.request.model_state = model_state;
bool call_ok = set_model_state_client.call(srv);
if(!call_ok)
{
ROS_ERROR("call service failed, model:%s", model_names[i].c_str());
}
else
{
if(!srv.response.success)
{
ROS_ERROR("model %s error: %s", model_names[i].c_str(), srv.response.status_message.c_str());
}
else
{
ROS_INFO("model %s, t=%.2f, y=%.3f", model_names[i].c_str(), t, y);
}
}

costmap_converter::ObstacleMsg obs;
obs.header = obs_array.header;
obs.polygon.points.resize(1);
obs.polygon.points[0].x = offset_x[i];
obs.polygon.points[0].y = y;
obs.radius = 0.15;
obs.id = i;
obs.velocities.twist.linear.y = amp[i] * omega[i] * cos(omega[i] * t + phase[i]);
obs_array.obstacles.push_back(obs);
}
obs_pub.publish(obs_array);
ros::spinOnce();
rate.sleep();
}
return 0;
}
