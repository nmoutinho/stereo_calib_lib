#include "images/imagesBase.h"

ProjectiveCamera::ProjectiveCamera(unsigned int resx, unsigned int resy, double fx, double fy, double cx, double cy, double k1, double k2, double k3, double p1, double p2)
{
	res.width  = resx;
	res.height = resy;

	intrinsics = Mat::zeros(3,3,CV_64F);
	intrinsics.at<double>(0,0) = fx;
	intrinsics.at<double>(1,1) = fy;
	intrinsics.at<double>(0,2) = cx;
	intrinsics.at<double>(1,2) = cy;
	intrinsics.at<double>(2,2) = 1;

	distortion = Mat::zeros(5,1,CV_64F);
	distortion.at<double>(0,0)=k1;
	distortion.at<double>(1,0)=k2;
	distortion.at<double>(2,0)=p1;
	distortion.at<double>(3,0)=p2;
	distortion.at<double>(4,0)=k3;
}

imagesBase::imagesBase(imagesBase_initial_parameters _imagesBase_initial_parameters_):
projectiveCameraLeft(_imagesBase_initial_parameters_.left_resx, _imagesBase_initial_parameters_.left_resy, _imagesBase_initial_parameters_.left_fx, _imagesBase_initial_parameters_.left_fy,
                     _imagesBase_initial_parameters_.left_cx, _imagesBase_initial_parameters_.left_cy, _imagesBase_initial_parameters_.left_k1, _imagesBase_initial_parameters_.left_k2,
                     _imagesBase_initial_parameters_.left_p1, _imagesBase_initial_parameters_.left_p2),
projectiveCameraRight(_imagesBase_initial_parameters_.right_resx, _imagesBase_initial_parameters_.right_resy, _imagesBase_initial_parameters_.right_fx, _imagesBase_initial_parameters_.right_fy,
                     _imagesBase_initial_parameters_.right_cx, _imagesBase_initial_parameters_.right_cy, _imagesBase_initial_parameters_.right_k1, _imagesBase_initial_parameters_.right_k2,
                     _imagesBase_initial_parameters_.right_p1, _imagesBase_initial_parameters_.right_p2)
{
    _imagesBase_initial_parameters = _imagesBase_initial_parameters_;

    Mat P1_left = cv::getOptimalNewCameraMatrix(projectiveCameraLeft.intrinsics, projectiveCameraLeft.distortion, projectiveCameraLeft.res, 0.0);

	cv::initUndistortRectifyMap(projectiveCameraLeft.intrinsics, projectiveCameraLeft.distortion, cv::Mat(), P1_left, projectiveCameraLeft.res, CV_16SC2, rectificationMap1_Left, rectificationMap2_Left);
	_imagesBase_data.calibMatLeft = P1_left;

    Mat P1_right = cv::getOptimalNewCameraMatrix(projectiveCameraRight.intrinsics, projectiveCameraRight.distortion, projectiveCameraRight.res, 0.0);

	cv::initUndistortRectifyMap(projectiveCameraRight.intrinsics, projectiveCameraRight.distortion, cv::Mat(), P1_right, projectiveCameraRight.res, CV_16SC2, rectificationMap1_Right, rectificationMap2_Right);
    _imagesBase_data.calibMatRight = P1_right;
}

imagesBase_data imagesBase::get_data()
{
    return _imagesBase_data;
};

imagesBase_data imagesBase::rectify(cv::Mat left_image, cv::Mat right_image)
{
    _imagesBase_data.originalLeftImage = left_image;
    _imagesBase_data.originalRightImage = right_image;

    cv::remap(left_image, _imagesBase_data.rectifiedLeftImage, rectificationMap1_Left, rectificationMap2_Left, cv::INTER_LINEAR);
    cv::remap(right_image, _imagesBase_data.rectifiedRightImage, rectificationMap1_Right, rectificationMap2_Right, cv::INTER_LINEAR);

    return _imagesBase_data;
};
