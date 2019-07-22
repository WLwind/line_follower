#include <cstdlib>
#include <line_follower/LineDetect.h>

int main(int argc, char **argv)
{
    bool display_img;
    // Initializing node and object
    ros::init(argc, argv, "detection");
    assert(argc!=1);
    if(strcmp(argv[1],"true")==0)
        display_img=true;
    else
        display_img=false;
    LineDetect det(display_img);
    ros::spin();
}