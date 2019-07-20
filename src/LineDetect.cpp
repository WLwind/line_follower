#include <line_follower/LineDetect.h>
#include <cstdlib>
#include <string>
#include <ros/console.h>

LineDetect::LineDetect()
{
  ROS_INFO("Create a window.");
  cv::namedWindow("Robot_View");//start a window to show images
  dir_msg.data=1;
  std::string color_string;
  if(ros::param::get("~line_color", color_string))
  {
    ROS_INFO("Line color is %s.",color_string.c_str());
    LowerColor = color_map[color_string][0];
    UpperColor = color_map[color_string][1];
  }
  else
  {
    ROS_INFO("Line color is red.");
  }
}

LineDetect::~LineDetect()
{
  ROS_INFO("Destroy the window.");
  cv::destroyWindow("Robot_View");
}

void LineDetect::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    img = cv_ptr->image;
    img_filt = Gauss(img);
    dir_msg.data = colorthresh(img_filt);
    dirPub.publish(dir_msg);
//    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());//format error
  }
}

cv::Mat LineDetect::Gauss(cv::Mat input)
{
  cv::Mat output;
// Applying Gaussian Filter
  cv::GaussianBlur(input, output, cv::Size(3, 3), 0.1, 0.1);
  return output;
}

int LineDetect::colorthresh(cv::Mat input)
{
  // Initializaing variables
  cv::Size s = input.size();
  std::vector<std::vector<cv::Point> > v;
  auto w = s.width;
  auto h = s.height;
  auto c_x = 0.0;
  // Detect all objects within the HSV range
  cv::cvtColor(input, img_hsv, CV_BGR2HSV);
  cv::inRange(img_hsv, LowerColor, UpperColor, img_mask);
  if(strColor=="red")//red has 2 sections
  {
    cv::Mat img_mask2;
    cv::inRange(img_hsv, color_map[strColor][2], color_map[strColor][3], img_mask2);
    img_mask=img_mask+img_mask2;
  }
  img_mask(cv::Rect(0, 0, w, 0.8*h)) = 0;
  // Find contours for better visualization
  cv::findContours(img_mask, v, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  // If contours exist add a bounding
  // Choosing contours with maximum area
  if (v.size() != 0) {
  auto area = 0;
  auto idx = 0;
  auto count = 0;
  while (count < v.size()) {
    if (area < v[count].size()) {
       idx = count;
       area = v[count].size();
    }
    count++;
  }
  cv::Rect rect = boundingRect(v[idx]);
  cv::Point pt1, pt2, pt3;
  pt1.x = rect.x;
  pt1.y = rect.y;
  pt2.x = rect.x + rect.width;
  pt2.y = rect.y + rect.height;
  pt3.x = pt1.x+5;
  pt3.y = pt1.y-5;
  // Drawing the rectangle using points obtained
  rectangle(input, pt1, pt2, CV_RGB(255, 0, 0), 2);
  // Inserting text box
  cv::putText(input, "Line Detected", pt3,
    CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0));
  }
  // Mask image to limit the future turns affecting the output
  img_mask(cv::Rect(0.7*w, 0, 0.3*w, h)) = 0;
  img_mask(cv::Rect(0, 0, 0.3*w, h)) = 0;
  // Perform centroid detection of line
  cv::Moments M = cv::moments(img_mask);
  if (M.m00 > 0) {
    cv::Point p1(M.m10/M.m00, M.m01/M.m00);
    cv::circle(img_mask, p1, 5, cv::Scalar(155, 200, 0), -1);
  }
  c_x = M.m10/M.m00;
  // Tolerance to chooise directions
  auto tol = 15;
  auto count = cv::countNonZero(img_mask);
  // Turn left if centroid is to the left of the image center minus tolerance
  // Turn right if centroid is to the right of the image center plus tolerance
  // Go straight if centroid is near image center
  if (c_x < w/2-tol) {
    dir = 0;
  } else if (c_x > w/2+tol) {
    dir = 2;
  } else {
    dir = 1;
  }
  // Search if no line detected
  if (count == 0) {
    dir = 3;
  }
  // Output images viewed by the turtlebot
  imshow("Robot_View", input);
  proc_img_msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8", input).toImageMsg();//CVimage to ROS image
  proc_img_pub.publish(proc_img_msg);//publish processed image
  return dir;
}
