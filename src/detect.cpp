#include <cstdlib>
#include <line_follower/LineDetect.h>

int main(int argc, char **argv)
{
    // Initializing node and object
    ros::init(argc, argv, "detection");
    LineDetect det;
    // Creating Publisher and subscriber
    // Closing image viewer
    ros::spin();
}