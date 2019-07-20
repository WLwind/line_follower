#include <line_follower/VelCmdRobot.h>
#include <ros/console.h>

int main(int argc, char **argv)
{
    // Initializing node and object
    ros::init(argc, argv, "Velocity");
    ros::NodeHandle n;
    VelCmdRobot bot;
    geometry_msgs::Twist velocity;
    // Creating subscriber and publisher
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        // Publish velocity commands to turtlebot
        bot.vel_cmd(velocity, vel_pub, rate);
        rate.sleep();
    }
    return 0;
}