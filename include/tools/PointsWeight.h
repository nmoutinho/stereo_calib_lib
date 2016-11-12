#ifndef _POINTS_WEIGHT_H_
#define _POINTS_WEIGHT_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include "filter/VisualOdometry_Base.h"

double PointWeight_ty(cv::Point left_point, cv::Point right_point, cv::Mat Kleft, cv::Mat Kright, cv::Mat T_left_to_right);
double PointWeight_ty(double x, double y, double z, double fx, double fy, double baseline, double ty, double min_displacement);
double PointWeight_ty(cv::Point left_point, cv::Point right_point, cv::Mat Kleft, cv::Mat Kright, double baseline, cv::Mat r_rect_left, cv::Mat r_rect_right);

double PointWeight_tz(cv::Point left_point, cv::Point right_point, cv::Mat Kleft, cv::Mat Kright, cv::Mat T_left_to_right);
double PointWeight_tz(double x, double y, double z, double fx, double fy, double baseline, double tz, double min_displacement);
double PointWeight_tz(cv::Point left_point, cv::Point right_point, cv::Mat Kleft, cv::Mat Kright, double baseline, cv::Mat r_rect_left, cv::Mat r_rect_right);

double PointWeight_rx(cv::Point point, cv::Mat K, int image_height);
double PointWeight_rx(double x, double y, double z, double fx, double fy, double baseline, double rx, double min_displacement);

double PointWeight_ry(cv::Point point, cv::Mat K, int image_width, int image_height);
double PointWeight_ry(double x, double y, double z, double fx, double fy, double baseline, double ry, double min_displacement);

double PointWeight_rz(cv::Point point, cv::Mat K, int image_width, int image_height);
double PointWeight_rz(double x, double y, double z, double fx, double fy, double baseline, double rz, double min_displacement);

#endif
