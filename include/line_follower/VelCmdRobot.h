#pragma once
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

class VelCmdRobot
{
public:
    VelCmdRobot();
    virtual ~VelCmdRobot(){}
    void dir_sub(const std_msgs::Int32ConstPtr& msg);
    void vel_cmd(geometry_msgs::Twist &velocity, ros::Publisher &pub, ros::Rate &rate);
    int getDir() const;
private:
    ros::NodeHandle nh;
    ros::Subscriber sub{nh.subscribe<std_msgs::Int32>("/direction", 1, boost::bind(&VelCmdRobot::dir_sub,this,_1))};
    int dir{1};
};
