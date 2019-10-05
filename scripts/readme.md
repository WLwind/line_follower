//功能
颜色跟踪，只能给出一个旋转的z方向速度（小车地盘标准），前进或者后退需要深度摄像机？
//运行
roslaunch usb_cam usb_cam-test.launch #发布图像信息
./pubopencv.py #接受图像信息处理，发布roi
./follerw.py #发布cmd_vel
//查看
rosrun rqt_plot rqt_plot
