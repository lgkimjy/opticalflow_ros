/*
 * opticalflow_ros.cpp
 *      Author: junyoung kim / lgkimjy
 */

#include <ros/ros.h>
#include <iostream>
#include <iostream>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Vector3.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <sys/stat.h>

using namespace cv;
using namespace std;

Mat frame, prvs;
bool initialized = false;

sensor_msgs::Image image_msg;

sensor_msgs::Image image2message(Mat image)
{
    //insert images to ros message
    cv_bridge::CvImage cv_image;
    cv_image.image = image;
    cv_image.encoding = "bgr8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);
    return ros_image;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
    
    ROS_INFO("[mission] callback functioning, OPENCV version : %s", CV_VERSION);

    try{
        frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        // cv::imshow("view", frame);
        // cv::waitKey(30);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    if(!initialized){
        cvtColor(frame, prvs, COLOR_BGR2GRAY);
        initialized = true;
    }
    else{
        Mat next;
        cvtColor(frame, next, COLOR_BGR2GRAY);
        Mat flow(prvs.size(), CV_32FC2);
        calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
        // visualization
        Mat flow_parts[2];
        split(flow, flow_parts);
        Mat magnitude, angle, magn_norm;
        cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
        normalize(magnitude, magn_norm, 0.0f, 1.0f, NORM_MINMAX);
        Mat oang = angle * 3.141592 / 180.0;
        angle *= ((1.f / 360.f) * (180.f / 255.f));

        //build hsv image
        Mat _hsv[3], hsv, hsv8, bgr;
        _hsv[0] = angle;
        _hsv[1] = Mat::ones(angle.size(), CV_32F);
        _hsv[2] = magn_norm;
        merge(_hsv, 3, hsv);
        hsv.convertTo(hsv8, CV_8U, 255.0);
        cvtColor(hsv8, bgr, COLOR_HSV2BGR);
        imshow("frame", bgr);

        // representation using vectors
        int step = 10;
        Mat img_vec = frame;
        for (int r=0; r<angle.rows; r+=step) {
            for (int c=0; c<angle.cols; c+=step){
            float ang = oang.at<float>(r,c);
            float m = magn_norm.at<float>(r,c) * 20.0;
            Point pt1 = cv::Point(c, r);
            Point pt2 = cv::Point(c + m * cos(ang) , r + m * sin(ang));
            line(img_vec, pt1, pt2, Scalar(0, 255, 0), 1, 8, 0); 
            }
        }
        imshow("frame3", img_vec);
        image_msg = image2message(img_vec);

        waitKey(30);
    prvs = next;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "misson");
	ros::NodeHandle n;
    ros::Rate loop_rate(100);

    ros::Subscriber image_sub = n.subscribe("/image_raw", 1, imageCallback);
    // ros::Subscriber image_compressed_sub = n.subscribe("/image_raw/compressed",1,CompressedImageCallback);
    ros::Publisher vector_pub = n.advertise<geometry_msgs::Vector3>("/processed_vector",1);
    ros::Publisher image_pub = n.advertise<sensor_msgs::Image>("/processed_image",1);

    while(ros::ok()){
        image_pub.publish(image_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}