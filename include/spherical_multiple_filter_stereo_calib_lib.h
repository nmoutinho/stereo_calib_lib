#ifndef _SPHERICAL_STEREO_MULTIPLE_FILTER_CALIB_LIB_H_
#define _SPHERICAL_STEREO_MULTIPLE_FILTER_CALIB_LIB_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "filter/calibrationSphericalMultipleFilterStereoCameras.h"
#include "filter/Kinematics.h"
#include "filter/VisualOdometry_Base.h"
#include "features/featuresSIFT.h"
#include "plot/Plot.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "filter/Jacobians.h"

struct spherical_multiple_filter_stereo_calib_params
{
    double baseline;
    double left_cam_resx;
    double left_cam_resy;
    double left_cam_fx;
    double left_cam_fy;
    double left_cam_cx;
    double left_cam_cy;
    double right_cam_resx;
    double right_cam_resy;
    double right_cam_fx;
    double right_cam_fy;
    double right_cam_cx;
    double right_cam_cy;
};

struct spherical_multiple_filter_stereo_calib_data
{
    cv::Mat transformation_left_cam_to_right_cam;
    cv::Mat R_left_cam_to_right_cam;
    cv::Mat rot_left_cam_to_right_cam;
    cv::Mat t_left_cam_to_right_cam;
};

struct spherical_multiple_filter_stereo_disparity_data
{
    cv::Mat disparity_image;
    cv::Mat disparity_values;
    cv::Mat point_cloud_xyz;
    cv::Mat point_cloud_rgb;
};

class spherical_multiple_filter_stereo_calib {

    public:

        spherical_multiple_filter_stereo_calib_params sscp_general;

        //variables
        double baseline;
        double number_fixed_state_params;
        double translation_state_noise;
        double rotation_state_noise;
        double translation_transition_noise;
        double rotation_transition_noise;
        double features_measurements_noise;
        double matching_threshold;
        double max_number_of_features;
        double min_number_of_features;
        double number_measurements;

        //filter state and covariance matrix
        cv::Mat X, P;

        //the calibration system
        calibrationSphericalMultipleFilterStereoCameras csc;

        cv::Mat Kleft, Kright;

        std::vector<double> plot_data_1;
        std::vector<double> plot_data_2;
        std::vector<double> plot_data_3;
        std::vector<double> plot_data_4;
        std::vector<double> plot_data_5;
        std::vector<double> plot_data_6;

	public:

		spherical_multiple_filter_stereo_calib(spherical_multiple_filter_stereo_calib_params sscp_general_);

        // -------> cameras_encoders = [left_y, right_y, left_x, right_x, left_z, right_z]'
		void calibrate(const cv::Mat image_left, const cv::Mat image_right);
		void calibrate(std::vector<Feature> features_left, std::vector<Feature> features_right);

		spherical_multiple_filter_stereo_calib_data get_calibrated_transformations();
        spherical_multiple_filter_stereo_disparity_data get_disparity_map(cv::Mat left_image, cv::Mat right_image);

};

#endif
