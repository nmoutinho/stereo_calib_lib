#ifndef _POINTS_WEIGHT_H_
#define _POINTS_WEIGHT_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>

double PointWeight_rx(cv::Point point, cv::Mat K, int image_height);
double PointWeight_ry(cv::Point point, cv::Mat K, int image_width);
double PointWeight_rz(cv::Point point, cv::Mat K, int image_width, int image_height);

#endif
