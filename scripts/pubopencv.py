#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image,RegionOfInterest
from std_msgs.msg import String
import imutils

class image_converter:
    def __init__(self):    
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.ROI = RegionOfInterest()
        self.roi_pub = rospy.Publisher("/roi", RegionOfInterest, queue_size=100)
        # 初始化 全局变量
      
        self.detect_box = None # 检测的目标区域位置框
        

    def callback(self,data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        #高斯模糊
        blurred = cv2.GaussianBlur(cv_image, (11,11), 0)
        #转换颜色空间到HSV
        hsv = cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)
        #定义黄色的HSV阀值
        lower  = np.array([20,100,100])
        upper = np.array([220,255,255])
        #对图片进行二值化处理
        mask = cv2.inRange(hsv, lower, upper)
        cv2.imshow("test", mask)
        #腐蚀操作
        #mask = cv2.erode(mask,None,iterations = 2)
        #膨胀操作，先腐蚀后膨胀以消除噪音
        #mask = cv2.dilate(mask,None,iterations=2)
        #寻找图中轮廓
        cnts = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        #至少存在一个轮廓进行以下操作
        if len(cnts) >0:
            c = max(cnts, key=cv2.contourArea)
            #使用最小外接圆圈出最大的轮廓
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            #计算轮廓的矩
            M = cv2.moments(c)
            #中心坐标
            #center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            #提取轮廓中的横,纵坐标
            #cx = int(M["m10"] / M["m00"])
            #cy = int(M["m10"] / M["m00"])
            #画出最小外界圆以及圆心
            if radius >5:
                cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(cv_image,(int(x), int(y)) , 5, (0, 0, 255), -1)
               
                #将数据保存为全局变量，并转化为/roi的数据格式
                self.detect_box = (x, y, x+radius/2.0, y+radius/2.0)

        # 显示Opencv格式的图像
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
        self.publish_roi()
    


 #发布区域
    def publish_roi(self):
        roi_box = self.detect_box
        #roi_box[0] = max(0, roi_box[0])
        #roi_box[1] = max(0, roi_box[1])    
        try:
            ROI = RegionOfInterest()
            ROI.x_offset = int(roi_box[0])
            ROI.y_offset = int(roi_box[1])
            ROI.width = int(roi_box[2])
            ROI.height = int(roi_box[3])
            rospy.loginfo(ROI)
            self.roi_pub.publish(ROI)
        except:
            rospy.loginfo("Publishing ROI failed")	        


if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Starting cv_bridge_test node")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
cv2.destroyAllWindows()
