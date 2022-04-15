#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <signal.h>

#include "opencv2/opencv.hpp"

// 管道配置
std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

// 信号回调函数
void signalHandler(int signum)
{
    ROS_INFO("%s is received, Terminating the node...", strsignal(signum));
    ros::shutdown();
    exit(signum);
}

int main(int argc, char **argv)
{
    int capture_width = 680;
    int capture_height = 480;
    int display_width = 680;
    int display_height = 480;
    int framerate = 15;
    int flip_method = 0;
    std::string pipeline = gstreamer_pipeline(capture_width,
                                         capture_height,
                                         display_width,
                                         display_height,
                                         framerate,
                                         flip_method);
    
    ros::init(argc, argv, "csi_camera"); // 初始化ROS节点 名为 Mono
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image_raw", 1);

    // 开启 csi 摄像头
    cv::VideoCapture capture(pipeline, cv::CAP_GSTREAMER);
    if (!capture.isOpened()){
        ROS_ERROR("Camera failed to turn on!!");
        ros::shutdown();
        exit(-1);
    }
    ROS_INFO("The camera has been opened!!");

    cv::Mat image;
    sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(15);  // 20帧
    signal(SIGINT, signalHandler);
    while (ros::ok()){

        /* 将 OpenCV 的 MAT 格式转为 ROS 的ImagePtr 格式 */
        capture >> image;
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
    // 释放管道
    capture.release();  
}