#pragma once
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <map>
#include <vector>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

/**
*@brief Line Detect class contains all the functions for image procesing and direction publishing
*/
class LineDetect {
 public:
    LineDetect();
    virtual ~LineDetect();
    cv::Mat img;  /// Input image in opencv matrix format
    cv::Mat img_filt;  /// Filtered image in opencv matrix format
    int dir;  /// Direction message to be published
/**
*@brief Callback used to subscribe to the image topic from the Turtlebot and convert to opencv image format
*@param msg is the image message for ROS
*@return none
*/
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
/**
*@brief Function that applies Gaussian filter in the input image 
*@param input is the image from the turtlebot in opencv matrix format
*@return Mat of Gaussian filtered image in opencv matrix format
*/
    cv::Mat Gauss(cv::Mat input);
/**
*@brief Function to perform line detection using color thresholding,image masking and centroid detection to publish direction 
*@param input is the Filtered input image in opencv matrix format
*@return int direction which returns the direction the turtlebot should head in
*/
    int colorthresh(cv::Mat input);

 private:
    ros::NodeHandle n;
    image_transport::ImageTransport ith{n};//image_transport handle
    cv::Scalar LowerColor{0 , 43, 46};//red section 1
    cv::Scalar UpperColor{10, 255, 255};
    cv::Scalar LowerRed{156,43,46};//red section 2
    cv::Scalar UpperRed{180,255,255};
    std::vector<cv::Scalar> color_range{LowerColor,UpperColor,LowerRed,UpperRed};
    cv::Mat img_hsv;
    cv::Mat img_mask;
    sensor_msgs::ImagePtr proc_img_msg;//processed image
    ros::Publisher dirPub{n.advertise<std_msgs::Int32>("/direction", 1)};
    ros::Subscriber sub{n.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw",1,boost::bind(&LineDetect::imageCallback,this,_1))};
    image_transport::Publisher proc_img_pub{ith.advertise("/robot_view_image", 1)};//processed image publisher
    std_msgs::Int32 dir_msg;
    std::map<std::string,std::vector<cv::Scalar>> color_map{{"red",color_range},{"yellow",std::vector<cv::Scalar>{cv::Scalar{26,43,46},cv::Scalar{34,255,255}}},{"green",std::vector<cv::Scalar>{cv::Scalar{35,43,46},cv::Scalar{77,255,255}}},{"blue",std::vector<cv::Scalar>{cv::Scalar{100,43,46},cv::Scalar{124,255,255}}},{"white",std::vector<cv::Scalar>{cv::Scalar{0,0,221},cv::Scalar{180,30,255}}},{"black",std::vector<cv::Scalar>{cv::Scalar{0,0,0},cv::Scalar{180,255,46}}}};//a dictionary of HSV color
    std::string strColor{"red"};
};