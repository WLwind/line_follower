#include <line_follower/VelCmdRobot.h>
#include <ros/console.h>

VelCmdRobot::VelCmdRobot()
{
    ROS_INFO("Create a VelCmdRobot instance.");
}

void VelCmdRobot::dir_sub(const std_msgs::Int32ConstPtr& msg)
{
    dir = msg->data;
}

void VelCmdRobot::vel_cmd(geometry_msgs::Twist &velocity, ros::Publisher &pub, ros::Rate &rate)
{
    // If direction is left
    if (dir == 0)
    {
        velocity.linear.x = 0.1;
        velocity.angular.z = 0.1;
        pub.publish(velocity);
        rate.sleep();
        ROS_INFO_STREAM("Turning Left");
    }
    // If direction is straight
    else if (dir == 1)
    {
        velocity.linear.x = 0.1;
        velocity.angular.z = 0;
        pub.publish(velocity);
        rate.sleep();
        ROS_INFO_STREAM("Straight");
    }
    // If direction is right
    else if (dir == 2)
    {
        velocity.linear.x = 0.1;
        velocity.angular.z = -0.1;
        pub.publish(velocity);
        rate.sleep();
        ROS_INFO_STREAM("Turning Right");
    }
    // If robot has to search
    else if (dir == 3)
    {
        velocity.linear.x = 0;
        velocity.angular.z = 0.15;
        pub.publish(velocity);
        rate.sleep();
        ROS_INFO_STREAM("Searching");
    }
}

int VelCmdRobot::getDir() const
{
    return dir;
}