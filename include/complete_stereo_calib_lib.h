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
    double right_cam_resx;
    double right_cam_resy;
    double right_cam_fx;
    double right_cam_fy;
    double right_cam_cx;
    double right_cam_cy;
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
        stereo_calib_params scp;

        stereo_calib sc;

        cv::Mat Kleft, Kright;

        bool use_good_points_only;

        std::vector<double> plot_data_1;
        std::vector<double> plot_data_2;
        std::vector<double> plot_data_3;
        std::vector<double> plot_data_4;
        std::vector<double> plot_data_5;
        std::vector<double> plot_data_6;

	public:

		complete_stereo_calib(complete_stereo_calib_params cscp_general_);

        // -------> cameras_encoders = [left_y, right_y, left_x, right_x, left_z, right_z]'
		void calibrate(const cv::Mat image_left, const cv::Mat image_right, const cv::Mat cameras_encoders);
		void calibrate(std::vector<Feature> features_left, std::vector<Feature> features_right, const cv::Mat cameras_encoders);

		cv::Mat get_offsets();

		complete_stereo_calib_data get_calibrated_transformations(cv::Mat cameras_encoders);
		complete_stereo_calib_data get_calibrated_transformations();

        complete_stereo_disparity_data get_disparity_map(cv::Mat left_image, cv::Mat right_image, cv::Mat cameras_encoders);
        complete_stereo_disparity_data get_disparity_map(cv::Mat left_image, cv::Mat right_image);

    private:

        void good_points_only(std::vector<Feature> features_left, std::vector<Feature> features_right,
complete_stereo_calib_data cscd, cv::Mat Kleft, cv::Mat Kright,
std::vector<Feature> &good_features_left, std::vector<Feature> &good_features_right);

        double feature_weight(cv::Point point_left, cv::Point point_right,
cv::Mat Kleft, cv::Mat Kright, cv::Mat T_Unified_2_left, cv::Mat T_Unified_2_right);

};

#endif
