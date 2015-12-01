#ifndef _SPHERICAL_STEREO_MULTIPLE_FILTER_CALIB_LIB_H_
#define _SPHERICAL_STEREO_MULTIPLE_FILTER_CALIB_LIB_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "filter/calibrationSphericalMultipleFilterStereoCameras.h"
#include "filter/Kinematics.h"
#include "filter/VisualOdometry_Base.h"
#include "features/featuresSIFT.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "filter/Jacobians.h"
#include "tools/PointsWeight.h"
#include "tools/ToString.h"
#include"tools/EyesStereoModel.h"

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
    double ty, tz, rx, ry, rz;
};

struct spherical_multiple_filter_stereo_disparity_data
{
    cv::Mat disparity_image;
    cv::Mat disparity_values;
    cv::Mat point_cloud_xyz;
    cv::Mat point_cloud_rgb;
};

struct filterMeasurementsStruct
{
    cv::Mat Z_ty;
    cv::Mat Z_tz;
    cv::Mat Z_rx;
    cv::Mat Z_ry;
    cv::Mat Z_rz;

    cv::Mat R_ty;
    cv::Mat R_tz;
    cv::Mat R_rx;
    cv::Mat R_ry;
    cv::Mat R_rz;

    cv::Mat dG_dZ;
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
        double translation_measurements_noise;
        double rotation_measurements_noise;
        double features_measurements_noise;
        double matching_threshold;
        double max_number_of_features;
        double min_number_of_features;
        double number_measurements;

        //filter state and covariance matrix
        cv::Mat X_ty, X_tz, X_rx, X_ry, X_rz;
        cv::Mat P_ty, P_tz, P_rx, P_ry, P_rz;

        //the calibration system
        calibrationSphericalMultipleFilterStereoCameras csc_ty, csc_tz, csc_rx, csc_ry, csc_rz;

        cv::Mat Kleft, Kright;

        std::vector<double> plot_data_1;
        std::vector<double> plot_data_2;
        std::vector<double> plot_data_3;
        std::vector<double> plot_data_4;
        std::vector<double> plot_data_5;
        std::vector<double> plot_data_6;

        cv::Mat previous_left_img;
        cv::Mat previous_right_img;
        bool first_images;
        double min_image_diff;
        bool use_measurements;
        bool filters_converged;

        bool use_good_points;
        bool use_close_points;

        bool use_good_points_ty;
        bool use_good_points_tz;
        bool use_good_points_rx;
        bool use_good_points_ry;
        bool use_good_points_rz;

        double resize_factor;

        int total_number_of_features;

	public:

		spherical_multiple_filter_stereo_calib(spherical_multiple_filter_stereo_calib_params sscp_general_);

        // -------> cameras_encoders = [left_y, right_y, left_x, right_x, left_z, right_z]'
		void calibrate(const cv::Mat image_left, const cv::Mat image_right);
		void calibrate(std::vector<Feature> features_left, std::vector<Feature> features_right);

		spherical_multiple_filter_stereo_calib_data get_calibrated_transformations();
        spherical_multiple_filter_stereo_disparity_data get_disparity_map(cv::Mat left_image, cv::Mat right_image);

        //functions
        filterMeasurementsStruct defineFiltersMeasurementsVector(double ty_pred, double tz_pred, double rx_pred, double ry_pred, double rz_pred,
                                                                 std::vector<Feature> features_left, std::vector<Feature> features_right, int number_of_features, double translation_noise,
                                                                 double rotation_noise, double features_noise);

        filterMeasurementsStruct defineFiltersMeasurementsVector(double ty_pred, double tz_pred, double rx_pred, double ry_pred, double rz_pred,
                                                                 std::vector<Feature> features_left, std::vector<Feature> features_right, int number_of_features, double ty_uncertainty, double tz_uncertainty,
                                                                 double rx_uncertainty, double ry_uncertainty, double rz_uncertainty, double features_noise);

};

#endif
