#ifndef _POINTS_WEIGHT_H_
#define _POINTS_WEIGHT_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include "filter/VisualOdometry_Base.h"

double PointWeight_ty(cv::Point left_point, cv::Point right_point, cv::Mat Kleft, cv::Mat Kright, cv::Mat T_left_to_right);
double PointWeight_tz(cv::Point left_point, cv::Point right_point, cv::Mat Kleft, cv::Mat Kright, cv::Mat T_left_to_right);
double PointWeight_rx(cv::Point point, cv::Mat K, int image_height);
double PointWeight_ry(cv::Point point, cv::Mat K, int image_width, int image_height);
double PointWeight_rz(cv::Point point, cv::Mat K, int image_width, int image_height);


#endif
