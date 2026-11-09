// path_show.cpp 只做可视化
#include <ros/ros.h>
#include <nav_msgs/Path.h>

ros::Publisher vis_pub;
void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    vis_pub.publish(*msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_show");
    ros::NodeHandle nh;
    vis_pub = nh.advertise<nav_msgs::Path>("/rviz_path",10);
    ros::Subscriber sub = nh.subscribe<nav_msgs::Path>("/show_path",10,pathCallback);
    ros::spin();
    return 0;
}
