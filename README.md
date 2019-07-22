# line_follower  
A ROS package that you can let your robot follow a line on the ground. You can draw a line or paste a piece of adhesive tape on the floor.
## Launch files
* line_follower.launch  
Run this launch file to start following.
## Arguments
1. rgb_image_topic  
The topic that published from a camera node (sensor_msgs/Image).  
2. cmd_vel_topic  
The topic published to the robot mobile base (geometry_msgs/Twist).  
3. display_img  
Whether or not to display the processed image in a window via OpenCV (default value is false).  
4. line_color  
The color of the line that the robot will follow. You can set this argument to one of the following strings (default line_color is red):  
![#f03c15](https://placehold.it/15/f03c15/000000?text=+) `red` ![#ffeb3b](https://placehold.it/15/ffeb3b/000000?text=+) `yellow` ![#259b24](https://placehold.it/15/259b24/000000?text=+) `green` ![#2a36b1](https://placehold.it/15/2a36b1/000000?text=+) `blue` ![#fafafa](https://placehold.it/15/fafafa/000000?text=+) `white` ![#000000](https://placehold.it/15/000000/000000?text=+) `black`  
__All the default argements are appropriate with Turtlebot2 and Kinect2__ 
## Published image topic  
* robot_view_image  
You can obtain the processed image by subscribing to this topic.  