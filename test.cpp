#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/opencv.hpp"
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "csi_camera"); // 初始化ROS节点 名为 Mono

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Pulisher pub = it.advertise("/camera/image_raw", 1);

    cv::VideoCapture cv::capture(0); // "/dev/video0"
    if (!cv::capture.isOpened()){
        ROS_ERROR("Camera failed to turn on!!");
        ros::shutdown();
        exit(-1)
    }
    ROS_INFO("The camera has been opened!!")

    cv::Mat image;
    sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(10);  // 20帧
    while (ros::ok()){

        /* 将 OpenCV 的 MAT 格式转为 ROS 的ImagePtr 格式 */
        
        cv::capture >> image;
        if (!image.empty()){
            // 封装消息内容
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            // 发布消息
            pub.publish(msg);
        }
        // 可接收回调函数 
        ros::spinOnce();

        // 保证在设定的 rate 进行
        loop_rate.sleep();
    }
}