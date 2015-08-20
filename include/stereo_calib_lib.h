#ifndef _STEREO_CALIB_LIB_H_
#define _STEREO_CALIB_LIB_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "filter/calibrationStereoCameras.h"
#include "filter/calibrationDynamicStereoCameras.h"
#include "filter/Kinematics.h"
#include "features/featuresSIFT.h"
//#include "features/featuresSIMULATED.h"
#include "plot/Plot.h"
#include "boost/date_time/posix_time/posix_time.hpp"

struct stereo_calib_params
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
    double matching_threshold;
    double min_number_of_features;
    double max_number_of_features;
    double number_fixed_state_params;
    double number_measurements;
    double encoders_state_noise;
    double encoders_transition_noise;
    double features_measurements_noise;
    double encoders_measurements_noise;
    bool calibrate_joint_0;
    bool calibrate_joint_1;
    bool calibrate_joint_2;
    bool calibrate_joint_3;
    bool calibrate_joint_4;
    bool calibrate_joint_5;
};

class stereo_calib {

    public:

        //filter state and covariance matrix
        cv::Mat X, P;

        //the calibration system
        //calibrationStereoCameras csc;
        calibrationDynamicStereoCameras csc;

        //the parameters for the system
        stereo_calib_params scp;

        //starting flag
        bool starting_flag;

        std::vector<double> plot_data_1;
        std::vector<double> plot_data_2;
        std::vector<double> plot_data_3;
        std::vector<double> plot_data_4;
        std::vector<double> plot_data_5;
        std::vector<double> plot_data_6;

	public:

		void initialize(stereo_calib_params scp_);

		void calibrate(std::vector<Feature> features_left, std::vector<Feature> features_right, cv::Mat encoders);
		void calibrate(std::vector<Feature> features_left, std::vector<Feature> features_right, cv::Mat encoders, cv::Mat dG_dZ);
        void calibrate(cv::Mat Z, cv::Mat R);
        void calibrate(cv::Mat Z, cv::Mat R, int NumFeatures, cv::Mat dG_dZ, bool encoders_measurements, bool features_measurements);

		cv::Mat get_offsets();
		cv::Mat get_offsets_covariance();
};

#endif
