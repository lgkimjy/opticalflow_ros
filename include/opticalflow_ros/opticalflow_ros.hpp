/*
 * opticalflow_ros.hpp
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