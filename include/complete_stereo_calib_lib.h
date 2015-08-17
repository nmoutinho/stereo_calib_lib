#ifndef _COMPLETE_STEREO_CALIB_LIB_H_
#define _COMPLETE_STEREO_CALIB_LIB_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "stereo_calib_lib.h"
#include "filter/Kinematics.h"
#include "filter/VisualOdometry_Base.h"
#include "features/featuresSIFT.h"
#include "plot/Plot.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "filter/Jacobians.h"

struct complete_stereo_calib_params
{
    double baseline;
    double left_cam_resx;
    double left_cam_resy;
    double left_cam_fx;
    double left_cam_fy;
    double left_cam_cx;
    double left_cam_cy;
    double left_cam_k1;
    double left_cam_k2;
    double left_cam_k3;
    double left_cam_k4;
    double right_cam_resx;
    double right_cam_resy;
    double right_cam_fx;
    double right_cam_fy;
    double right_cam_cx;
    double right_cam_cy;
    double right_cam_k1;
    double right_cam_k2;
    double right_cam_k3;
    double right_cam_k4;
    double lambda;
};

struct complete_stereo_calib_data
{
    cv::Mat Offsets;
    cv::Mat transformation_left_cam_to_right_cam;
    cv::Mat R_left_cam_to_right_cam;
    cv::Mat t_left_cam_to_right_cam;

    cv::Mat transformation_right_cam_to_left_cam;
    cv::Mat R_right_cam_to_left_cam;
    cv::Mat t_right_cam_to_left_cam;

    cv::Mat transformation_left_cam_to_baseline_center;
    cv::Mat transformation_right_cam_to_baseline_center;
};

struct complete_stereo_calib_measurements_data
{
    cv::Mat Z0; //all the measurements -> Encoders Features Left and Features Right
    cv::Mat Z1; //all the measurements -> Encoders Features Left and Features Right
    cv::Mat Z2; //all the measurements -> Encoders Features Left and Features Right
    cv::Mat Z3; //all the measurements -> Encoders Features Left and Features Right
    cv::Mat Z4; //all the measurements -> Encoders Features Left and Features Right

    cv::Mat R0; // the measurements covariance matrix adapted for RLY
    cv::Mat R1; // the measurements covariance matrix adapted for RRY
    cv::Mat R2; // the measurements covariance matrix adapted for RLX
    cv::Mat R3; // the measurements covariance matrix adapted for RLZ
    cv::Mat R4; // the measurements covariance matrix adapted for RRX

    cv::Mat dG_dZ;
};

struct complete_stereo_disparity_data
{
    cv::Mat disparity_image;
    cv::Mat disparity_values;
    cv::Mat point_cloud_xyz;
    cv::Mat point_cloud_rgb;
};

class complete_stereo_calib {

    public:

        complete_stereo_calib_params cscp_general;
        stereo_calib_params scp_general;
        stereo_calib_params scp0, scp1, scp2, scp3, scp4;

        stereo_calib sc0, sc1, sc2, sc3, sc4;

        cv::Mat Kleft, Kright;

        double offset_0, offset_1, offset_2, offset_3, offset_4;
        bool first_iteration, use_good_points_only;

        std::vector<double> plot_data_1;
        std::vector<double> plot_data_2;
        std::vector<double> plot_data_3;
        std::vector<double> plot_data_4;
        std::vector<double> plot_data_5;
        std::vector<double> plot_data_6;

	public:

		complete_stereo_calib(complete_stereo_calib_params cscp_general_);

        // -------> cameras_encoders = [left_y, right_y, left_x, right_x, left_z, right_z]'
		void calibrate(cv::Mat image_left, cv::Mat image_right, cv::Mat cameras_encoders);
		void calibrate(std::vector<Feature> features_left, std::vector<Feature> features_right, cv::Mat cameras_encoders);
		cv::Mat get_offsets();
		complete_stereo_calib_data get_calibrated_transformations(cv::Mat cameras_encoders);
        complete_stereo_disparity_data get_disparity_map(cv::Mat left_image, cv::Mat right_image, cv::Mat cameras_encoders);

    private:

        complete_stereo_calib_measurements_data get_measurements_data(
        std::vector<Feature> features_left, std::vector<Feature> features_right,
        cv::Mat encoders_0, cv::Mat encoders_1, cv::Mat encoders_2, cv::Mat encoders_3,
        cv::Mat encoders_4, double updated_encoders_measurements_noise, complete_stereo_calib_data cscd);

        void good_points_only(std::vector<Feature> features_left, std::vector<Feature> features_right,
complete_stereo_calib_data cscd, cv::Mat Kleft, cv::Mat Kright,
std::vector<Feature> &good_features_left, std::vector<Feature> &good_features_right);

        double feature_weight(cv::Point point_left, cv::Point point_right,
cv::Mat Kleft, cv::Mat Kright, cv::Mat T_Unified_2_left, cv::Mat T_Unified_2_right);
        double feature_weight_rx(cv::Point Point, cv::Mat K);
        double feature_weight_ry(cv::Point Point, cv::Mat K);
        double feature_weight_rz(cv::Point Point, cv::Mat K);

};

#endif
