#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import RegionOfInterest, CameraInfo
from geometry_msgs.msg import Twist
import thread

class ObjectTracker():
    def __init__(self):
        rospy.init_node("object_tracker")
                
        # 节点关闭 Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)

        # 更新频率 How often should we update the robot's motion?
        self.rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(self.rate)
        # 移动底盘最大旋转速度 The maximum rotation speed in radians per second
        self.max_rotation_speed = rospy.get_param("~max_rotation_speed", 2.0)       
        # 移动底盘最小旋转速度 The minimum rotation speed in radians per second
        self.min_rotation_speed = rospy.get_param("~min_rotation_speed", 0.5)        
        # Sensitivity to target displacements.  Setting this too high
        # can lead to oscillations of the robot.
        self.gain = rospy.get_param("~gain", 2.0) # 灵敏度，增益 相当于一个比例控制器         
        # The x threshold (% of image width) indicates how far off-center
        # the ROI needs to be in the x-direction before we react
        self.x_threshold = rospy.get_param("~x_threshold", 0.1) # 偏移阈值


        self.move_cmd = Twist()
        # Publisher to control the robot's movement  发布运动信息控制指令
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        
        
        # Wait for the camera_info topic to become available 等待
        rospy.loginfo("Waiting for camera_info topic...")
        rospy.wait_for_message("/usb_cam/camera_info", CameraInfo)        
        #订阅话题，获取相机图像信息 Subscribe the camera_info topic to get the image width and height
        rospy.Subscriber("/usb_cam/camera_info", CameraInfo, self.get_camera_info, queue_size=1)

       # 订阅ROI话题 Subscribe to the ROI topic and set the callback to update the robot's motion
	    # 回调函数为 set_cmd_ve()
        rospy.Subscriber("/roi", RegionOfInterest, self.set_cmd_vel, queue_size=100)
        
        # 等待ROI信息 Wait until we have an ROI to follow
        rospy.loginfo("Waiting for messages on /roi...")
        rospy.wait_for_message("/roi", RegionOfInterest)
 
        rospy.loginfo("ROI messages detected. Starting tracker...")
        
	    
    #  订阅ROI话题  的回调函数=================
    def set_cmd_vel(self, msg):
            
        # Compute the displacement of the ROI from the center of the image
	    # roi 中心 和 图像 中心的 水平方向偏移量=======================
        target_offset_x = msg.x_offset + msg.width / 2 - self.image_width / 2
        # 计算一个偏移比例    
        percent_offset_x = float(target_offset_x) / (float(self.image_width) / 2.0)
        # Rotate the robot only if the displacement of the target exceeds the threshold
	    # 直到 偏移比例 小于阈值=====
        if abs(percent_offset_x) > self.x_threshold:
            # Set the rotation speed proportional to the displacement of the target
            speed = self.gain * percent_offset_x # 相当于一个比例控制器 
            if speed < 0:
                direction = -1  #方向
            else:
                direction = 1
            # 旋转角速度
            move_cmd = Twist()
            move_cmd.angular.z = -direction * max(self.min_rotation_speed,min(self.max_rotation_speed, abs(speed)))
            self.cmd_vel_pub.publish(move_cmd)
            rospy.loginfo(move_cmd)
        
        self.move_cmd = Twist()# 调节完毕，不动
        


    

    def get_camera_info(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)     

if __name__ == '__main__':
    try:
        ObjectTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Object tracking node terminated.")