#ifndef _IMAGES_BASE_H_
#define _IMAGES_BASE_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;

struct imagesBase_data
{
        Mat originalLeftImage;
        Mat originalRightImage;

        Mat rectifiedLeftImage;
        Mat rectifiedRightImage;

        Mat calibMatLeft;
        Mat calibMatRight;
};

struct imagesBase_initial_parameters
{
    int left_resx;
    int left_resy;
    double left_cx;
    double left_cy;
    double left_fx;
    double left_fy;
    double left_k1;
    double left_k2;
    double left_k3;
    double left_k4;
    double left_k5;
    double left_p1;
    double left_p2;

    int right_resx;
    int right_resy;
    double right_cx;
    double right_cy;
    double right_fx;
    double right_fy;
    double right_k1;
    double right_k2;
    double right_k3;
    double right_k4;
    double right_k5;
    double right_p1;
    double right_p2;
};

struct ProjectiveCamera
{
	cv::Size res;
	cv::Mat intrinsics;
	cv::Mat distortion;

	ProjectiveCamera(unsigned int resx, unsigned int resy, double fx, double fy, double cx, double cy, double k1=0, double k2=0, double k3=0, double k4=0, double k5=0);
};

class imagesBase
{
    public:
    imagesBase_initial_parameters _imagesBase_initial_parameters;
    imagesBase_data _imagesBase_data;

    ProjectiveCamera projectiveCameraLeft;
    ProjectiveCamera projectiveCameraRight;

    Mat rectificationMap1_Left;
    Mat rectificationMap2_Left;
    Mat rectificationMap1_Right;
    Mat rectificationMap2_Right;

    imagesBase(imagesBase_initial_parameters _imagesBase_initial_parameters_);
    imagesBase_data get_data();
    imagesBase_data rectify(cv::Mat left_image, cv::Mat right_image);
};

#endif
