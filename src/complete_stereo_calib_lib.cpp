#include "complete_stereo_calib_lib.h"

using namespace std;
using namespace cv;

complete_stereo_calib::complete_stereo_calib(complete_stereo_calib_params cscp_general_)
{
    //set the parameters for the system
    cscp_general = cscp_general_;

    //the standard image is 320x240
    double resize_factor = 320./cscp_general.left_cam_resx;

    scp_general.baseline = cscp_general.baseline;
    scp_general.left_cam_resx = cscp_general.left_cam_resx;
    scp_general.left_cam_resy = cscp_general.left_cam_resy;
    scp_general.left_cam_cx = cscp_general.left_cam_cx;
    scp_general.left_cam_cy = cscp_general.left_cam_cy;
    scp_general.left_cam_fx = cscp_general.left_cam_fx;
    scp_general.left_cam_fy = cscp_general.left_cam_fy;
    scp_general.right_cam_resx = cscp_general.right_cam_resx;
    scp_general.right_cam_resy = cscp_general.right_cam_resy;
    scp_general.right_cam_cx = cscp_general.right_cam_cx;
    scp_general.right_cam_cy = cscp_general.right_cam_cy;
    scp_general.right_cam_fx = cscp_general.right_cam_fx;
    scp_general.right_cam_fy = cscp_general.right_cam_fy;
    scp_general.encoders_measurements_noise = 0.025; //0.0000000174;
    scp_general.encoders_state_noise = 0.5; //1.0;
    scp_general.encoders_transition_noise = 0.025; //0.025;
    scp_general.features_measurements_noise = 5/(resize_factor*resize_factor); //5;
    scp_general.matching_threshold = 0.3; //0.25;
    scp_general.max_number_of_features = 100;
    scp_general.min_number_of_features = 1;
    scp_general.number_measurements = 6;

    cscp_general.lambda = 0; //600;

    Kleft = Mat::eye(3,3,CV_64F);
    Kleft.at<double>(0,0) = scp_general.left_cam_fx;
    Kleft.at<double>(1,1) = scp_general.left_cam_fy;
    Kleft.at<double>(0,2) = scp_general.left_cam_cx;
    Kleft.at<double>(1,2) = scp_general.left_cam_cy;

    Kright = Mat::eye(3,3,CV_64F);
    Kright.at<double>(0,0) = scp_general.right_cam_fx;
    Kright.at<double>(1,1) = scp_general.right_cam_fy;
    Kright.at<double>(0,2) = scp_general.right_cam_cx;
    Kright.at<double>(1,2) = scp_general.right_cam_cy;

    scp0 = scp_general;
    scp0.number_fixed_state_params = 1;
    scp0.calibrate_joint_0 = true;
    scp0.calibrate_joint_1 = false;
    scp0.calibrate_joint_2 = false;
    scp0.calibrate_joint_3 = false;
    scp0.calibrate_joint_4 = false;
    scp0.calibrate_joint_5 = false;

    scp1 = scp_general;
    scp1.number_fixed_state_params = 1;
    scp1.calibrate_joint_0 = false;
    scp1.calibrate_joint_1 = true;
    scp1.calibrate_joint_2 = false;
    scp1.calibrate_joint_3 = false;
    scp1.calibrate_joint_4 = false;
    scp1.calibrate_joint_5 = false;

    scp2 = scp_general;
    scp2.number_fixed_state_params = 1;
    scp2.calibrate_joint_0 = false;
    scp2.calibrate_joint_1 = false;
    scp2.calibrate_joint_2 = true;
    scp2.calibrate_joint_3 = false;
    scp2.calibrate_joint_4 = false;
    scp2.calibrate_joint_5 = false;

    scp3 = scp_general;
    scp3.number_fixed_state_params = 1;
    scp3.calibrate_joint_0 = false;
    scp3.calibrate_joint_1 = false;
    scp3.calibrate_joint_2 = false;
    scp3.calibrate_joint_3 = false;
    scp3.calibrate_joint_4 = true;
    scp3.calibrate_joint_5 = false;

    scp4 = scp_general;
    scp4.number_fixed_state_params = 1;
    scp4.calibrate_joint_0 = false;
    scp4.calibrate_joint_1 = false;
    scp4.calibrate_joint_2 = false;
    scp4.calibrate_joint_3 = false;
    scp4.calibrate_joint_4 = false;
    scp4.calibrate_joint_5 = true;

    sc0.initialize(scp0);
    sc1.initialize(scp1);
    sc2.initialize(scp2);
    sc3.initialize(scp3);
    sc4.initialize(scp4);

    first_iteration = true;
    use_good_points_only = false;

    // OFFSETS - [ 0: LEFT_Y | 1: RIGHT_Y | 2: LEFT_X | 3: LEFT_Z | 4: RIGHT_Z ]
    offset_0 = 0;
	offset_1 = 0;
	offset_2 = 0;
	offset_3 = 0;
	offset_4 = 0;
}

void complete_stereo_calib::calibrate(cv::Mat image_left, cv::Mat image_right, cv::Mat cameras_encoders)
{
    featuresSIFT get_features;
    std::vector<Feature> features_left;
    std::vector<Feature> features_right;

    get_features.Apply(image_left, image_right, features_left, features_right, scp_general.max_number_of_features, scp_general.matching_threshold);

    calibrate(features_left, features_right, cameras_encoders);
}

void complete_stereo_calib::calibrate(std::vector<Feature> features_left, std::vector<Feature> features_right, cv::Mat cameras_encoders)
{
    if(first_iteration)
    {
        offset_0 = cameras_encoders.at<double>(0,0);
        offset_1 = cameras_encoders.at<double>(1,0);
        offset_2 = cameras_encoders.at<double>(2,0);
        offset_3 = cameras_encoders.at<double>(4,0);
        offset_4 = cameras_encoders.at<double>(5,0);

        first_iteration = false;
    }

    Mat real_encoders_0 = Mat::zeros(scp_general.number_measurements,1,CV_64F);
    real_encoders_0.at<double>(0,0) = cameras_encoders.at<double>(0,0);
    real_encoders_0.at<double>(1,0) = cameras_encoders.at<double>(1,0)-offset_1;
    real_encoders_0.at<double>(2,0) = cameras_encoders.at<double>(2,0)-offset_2;
    real_encoders_0.at<double>(4,0) = cameras_encoders.at<double>(4,0)-offset_3;
    real_encoders_0.at<double>(5,0) = cameras_encoders.at<double>(5,0)-offset_4;

    Mat real_encoders_1 = Mat::zeros(scp_general.number_measurements,1,CV_64F);
    real_encoders_1.at<double>(0,0) = cameras_encoders.at<double>(0,0)-offset_0;
    real_encoders_1.at<double>(1,0) = cameras_encoders.at<double>(1,0);
    real_encoders_1.at<double>(2,0) = cameras_encoders.at<double>(2,0)-offset_2;
    real_encoders_1.at<double>(4,0) = cameras_encoders.at<double>(4,0)-offset_3;
    real_encoders_1.at<double>(5,0) = cameras_encoders.at<double>(5,0)-offset_4;

    Mat real_encoders_2 = Mat::zeros(scp_general.number_measurements,1,CV_64F);
    real_encoders_2.at<double>(0,0) = cameras_encoders.at<double>(0,0)-offset_0;
    real_encoders_2.at<double>(1,0) = cameras_encoders.at<double>(1,0)-offset_1;
    real_encoders_2.at<double>(2,0) = cameras_encoders.at<double>(2,0);
    real_encoders_2.at<double>(4,0) = cameras_encoders.at<double>(4,0)-offset_3;
    real_encoders_2.at<double>(5,0) = cameras_encoders.at<double>(5,0)-offset_4;

    Mat real_encoders_3 = Mat::zeros(scp_general.number_measurements,1,CV_64F);
    real_encoders_3.at<double>(0,0) = cameras_encoders.at<double>(0,0)-offset_0;
    real_encoders_3.at<double>(1,0) = cameras_encoders.at<double>(1,0)-offset_1;
    real_encoders_3.at<double>(2,0) = cameras_encoders.at<double>(2,0)-offset_2;
    real_encoders_3.at<double>(4,0) = cameras_encoders.at<double>(4,0);
    real_encoders_3.at<double>(5,0) = cameras_encoders.at<double>(5,0)-offset_4;

    Mat real_encoders_4 = Mat::zeros(scp_general.number_measurements,1,CV_64F);
    real_encoders_4.at<double>(0,0) = cameras_encoders.at<double>(0,0)-offset_0;
    real_encoders_4.at<double>(1,0) = cameras_encoders.at<double>(1,0)-offset_1;
    real_encoders_4.at<double>(2,0) = cameras_encoders.at<double>(2,0)-offset_2;
    real_encoders_4.at<double>(4,0) = cameras_encoders.at<double>(4,0)-offset_3;
    real_encoders_4.at<double>(5,0) = cameras_encoders.at<double>(5,0);

    Mat offsets_covariance = Mat();
    offsets_covariance.push_back(sc0.get_offsets_covariance());
    offsets_covariance.push_back(sc1.get_offsets_covariance());
    offsets_covariance.push_back(sc2.get_offsets_covariance());
    offsets_covariance.push_back(sc3.get_offsets_covariance());
    offsets_covariance.push_back(sc4.get_offsets_covariance());

    double min, max;
    minMaxLoc(offsets_covariance, &min, &max);
    double updated_encoders_measurements_noise = sqrt(max);

    double uncertainty = updated_encoders_measurements_noise/scp_general.encoders_state_noise;
    if(uncertainty <= 0.1)
        use_good_points_only = true;

    //cout << "using good points: " << use_good_points_only << endl;
    //cout << "covariance: " << updated_encoders_measurements_noise*180/CV_PI << endl;

    sc0.scp.encoders_measurements_noise = updated_encoders_measurements_noise;
    sc1.scp.encoders_measurements_noise = updated_encoders_measurements_noise;
    sc2.scp.encoders_measurements_noise = updated_encoders_measurements_noise;
    sc3.scp.encoders_measurements_noise = updated_encoders_measurements_noise;
    sc4.scp.encoders_measurements_noise = updated_encoders_measurements_noise;

    complete_stereo_calib_data cscd = get_calibrated_transformations(cameras_encoders);

    std::vector<Feature> good_features_left, good_features_right;
    if(use_good_points_only)
        good_points_only(features_left, features_right, cscd, Kleft, Kright, good_features_left, good_features_right);
    else
    {
        good_features_left = features_left;
        good_features_right = features_right;
    }

    //cout << "# points used: " << good_features_left.size() << "/" << features_left.size() << endl << endl;

    complete_stereo_calib_measurements_data cscmd = get_measurements_data(good_features_left,
    good_features_right, real_encoders_0, real_encoders_1, real_encoders_2, real_encoders_3,
    real_encoders_4, updated_encoders_measurements_noise, cscd);

    bool encoders_measurements = true;
    bool features_measurements = false;

    if(good_features_left.size() >= scp_general.min_number_of_features)
    {
        features_measurements = true;
    }

    sc0.calibrate(cscmd.Z0, cscmd.R0, good_features_left.size(), cscmd.dG_dZ, encoders_measurements, features_measurements);
    sc1.calibrate(cscmd.Z1, cscmd.R1, good_features_left.size(), cscmd.dG_dZ, encoders_measurements, features_measurements);
    sc2.calibrate(cscmd.Z2, cscmd.R2, good_features_left.size(), cscmd.dG_dZ, encoders_measurements, features_measurements);
    sc3.calibrate(cscmd.Z3, cscmd.R3, good_features_left.size(), cscmd.dG_dZ, encoders_measurements, features_measurements);
    sc4.calibrate(cscmd.Z4, cscmd.R4, good_features_left.size(), cscmd.dG_dZ, encoders_measurements, features_measurements);

    offset_0 = sc0.get_offsets().at<double>(0,0);
    offset_1 = sc1.get_offsets().at<double>(0,0);
    offset_2 = sc2.get_offsets().at<double>(0,0);
    offset_3 = sc3.get_offsets().at<double>(0,0);
    offset_4 = sc4.get_offsets().at<double>(0,0);
}

Mat complete_stereo_calib::get_offsets()
{
    Mat offsets(5,1,CV_64F);
    offsets.at<double>(0,0) = offset_0;
    offsets.at<double>(1,0) = offset_1;
    offsets.at<double>(2,0) = offset_2;
    offsets.at<double>(3,0) = offset_3;
    offsets.at<double>(4,0) = offset_4;

    return offsets;
}

void complete_stereo_calib::good_points_only(std::vector<Feature> features_left, std::vector<Feature> features_right,
complete_stereo_calib_data cscd, cv::Mat Kleft, cv::Mat Kright,
std::vector<Feature> &good_features_left, std::vector<Feature> &good_features_right)
{
    for(int j=0; j<features_left.size(); j++)
    {
        double w = feature_weight(features_left[j].Point, features_right[j].Point, Kleft, Kright,
        cscd.transformation_left_cam_to_baseline_center.inv(),
        cscd.transformation_right_cam_to_baseline_center.inv());

        if(w == 1)
        {
            good_features_left.push_back(features_left[j]);
            good_features_right.push_back(features_right[j]);
        }
    }
}

complete_stereo_calib_measurements_data complete_stereo_calib::get_measurements_data(
std::vector<Feature> features_left, std::vector<Feature> features_right, cv::Mat encoders_0,
cv::Mat encoders_1, cv::Mat encoders_2, cv::Mat encoders_3, cv::Mat encoders_4,
double updated_encoders_measurements_noise, complete_stereo_calib_data cscd)
{
    complete_stereo_calib_measurements_data cscmd;

    int NumPoints = features_left.size();
    int Z_size = scp_general.number_measurements+4*NumPoints;
    Mat dG_dZ = Mat::zeros(NumPoints,Z_size,CV_64F);
    Mat dg_dE, dg_dFL, dg_dFR;

    cscmd.Z0 = Mat(Z_size,1,CV_64F);
    cscmd.Z1 = Mat(Z_size,1,CV_64F);
    cscmd.Z2 = Mat(Z_size,1,CV_64F);
    cscmd.Z3 = Mat(Z_size,1,CV_64F);
    cscmd.Z4 = Mat(Z_size,1,CV_64F);

    cscmd.R0 = Mat::zeros(Z_size, Z_size,CV_64F);
    cscmd.R1 = Mat::zeros(Z_size, Z_size,CV_64F);
    cscmd.R2 = Mat::zeros(Z_size, Z_size,CV_64F);
    cscmd.R3 = Mat::zeros(Z_size, Z_size,CV_64F);
    cscmd.R4 = Mat::zeros(Z_size, Z_size,CV_64F);

    Mat Z_EKplus1(scp_general.number_measurements,1,CV_64F);
    Mat Z_FLKplus1(2*NumPoints,1,CV_64F);
    Mat Z_FRKplus1(2*NumPoints,1,CV_64F);

    for (int i=0; i<scp_general.number_measurements; i++)
    {
        Z_EKplus1.at<double>(i,0) = encoders_0.at<double>(i,0);

        cscmd.Z0.at<double>(i,0) = encoders_0.at<double>(i,0);
        cscmd.Z1.at<double>(i,0) = encoders_1.at<double>(i,0);
        cscmd.Z2.at<double>(i,0) = encoders_2.at<double>(i,0);
        cscmd.Z3.at<double>(i,0) = encoders_3.at<double>(i,0);
        cscmd.Z4.at<double>(i,0) = encoders_4.at<double>(i,0);

        cscmd.R0.at<double>(i,i) = updated_encoders_measurements_noise*updated_encoders_measurements_noise;
        cscmd.R1.at<double>(i,i) = updated_encoders_measurements_noise*updated_encoders_measurements_noise;
        cscmd.R2.at<double>(i,i) = updated_encoders_measurements_noise*updated_encoders_measurements_noise;
        cscmd.R3.at<double>(i,i) = updated_encoders_measurements_noise*updated_encoders_measurements_noise;
        cscmd.R4.at<double>(i,i) = updated_encoders_measurements_noise*updated_encoders_measurements_noise;
    }

    /*Mat il = Mat::zeros(scp_general.left_cam_resy, 3*scp_general.left_cam_resx, CV_8UC3);
    Mat ir = Mat::zeros(scp_general.left_cam_resy, 3*scp_general.left_cam_resx, CV_8UC3);//*/

    for(int j=0; j<NumPoints; j++)
    {
        Z_FLKplus1.at<double>(2*j,0) = features_left[j].Point.x;
        Z_FLKplus1.at<double>(2*j+1,0) = features_left[j].Point.y;
        Z_FRKplus1.at<double>(2*j,0) = features_right[j].Point.x;
        Z_FRKplus1.at<double>(2*j+1,0) = features_right[j].Point.y;

        //double w = 1;/*feature_weight(features_left[j].Point, features_right[j].Point, Kleft, Kright,
        /*cscd.transformation_left_cam_to_baseline_center.inv(),
        cscd.transformation_right_cam_to_baseline_center.inv());//*/

        /*double w0 = 1; //feature_weight_ry(features_left[j].Point, Kleft);
        double w1 = 1; //w0; //feature_weight_ry(features_right[j].Point, Kright); //w0;
        double w2 = 1; //feature_weight_rx(features_left[j].Point, Kleft);
        double w3 = 1; //feature_weight_rz(features_left[j].Point, Kleft);
        double w4 = 1; //w3; //feature_weight_rz(features_right[j].Point, Kright); //w3;

        double features_noise_0 = (cscp_general.lambda*(1-w0) + 1)*scp_general.features_measurements_noise;
        double features_noise_1 = (cscp_general.lambda*(1-w1) + 1)*scp_general.features_measurements_noise;
        double features_noise_2 = (cscp_general.lambda*(1-w2) + 1)*scp_general.features_measurements_noise;
        double features_noise_3 = (cscp_general.lambda*(1-w3) + 1)*scp_general.features_measurements_noise;
        double features_noise_4 = (cscp_general.lambda*(1-w4) + 1)*scp_general.features_measurements_noise;//*/

        double features_noise_0 = scp_general.features_measurements_noise;
        double features_noise_1 = scp_general.features_measurements_noise;
        double features_noise_2 = scp_general.features_measurements_noise;
        double features_noise_3 = scp_general.features_measurements_noise;
        double features_noise_4 = scp_general.features_measurements_noise;//*/


        /*il.at<Vec3b>(features_left[j].Point.y, features_left[j].Point.x)[1] = w2*255;
        il.at<Vec3b>(features_left[j].Point.y, features_left[j].Point.x)[2] = (1-w2)*255;
        il.at<Vec3b>(features_left[j].Point.y, scp_general.left_cam_resx + features_left[j].Point.x)[1] = w0*255;
        il.at<Vec3b>(features_left[j].Point.y, scp_general.left_cam_resx + features_left[j].Point.x)[2] = (1-w0)*255;
        il.at<Vec3b>(features_left[j].Point.y, 2*scp_general.left_cam_resx + features_left[j].Point.x)[1] = w3*255;
        il.at<Vec3b>(features_left[j].Point.y, 2*scp_general.left_cam_resx + features_left[j].Point.x)[2] = (1-w3)*255;

        ir.at<Vec3b>(features_right[j].Point.y, features_right[j].Point.x)[1] = w2*255;
        ir.at<Vec3b>(features_right[j].Point.y, features_right[j].Point.x)[2] = (1-w2)*255;
        ir.at<Vec3b>(features_right[j].Point.y, scp_general.left_cam_resx + features_right[j].Point.x)[1] = w0*255;
        ir.at<Vec3b>(features_right[j].Point.y, scp_general.left_cam_resx + features_right[j].Point.x)[2] = (1-w0)*255;
        ir.at<Vec3b>(features_right[j].Point.y, 2*scp_general.left_cam_resx + features_right[j].Point.x)[1] = w3*255;
        ir.at<Vec3b>(features_right[j].Point.y, 2*scp_general.left_cam_resx + features_right[j].Point.x)[2] = (1-w3)*255;//*/


        cscmd.Z0.at<double>(scp_general.number_measurements+2*j,0) = features_left[j].Point.x;
        cscmd.Z0.at<double>(scp_general.number_measurements+2*j+1,0) = features_left[j].Point.y;
        cscmd.Z0.at<double>(scp_general.number_measurements+2*NumPoints+2*j,0) = features_right[j].Point.x;
        cscmd.Z0.at<double>(scp_general.number_measurements+2*NumPoints+2*j+1,0) = features_right[j].Point.y;

        cscmd.Z1.at<double>(scp_general.number_measurements+2*j,0) = features_left[j].Point.x;
        cscmd.Z1.at<double>(scp_general.number_measurements+2*j+1,0) = features_left[j].Point.y;
        cscmd.Z1.at<double>(scp_general.number_measurements+2*NumPoints+2*j,0) = features_right[j].Point.x;
        cscmd.Z1.at<double>(scp_general.number_measurements+2*NumPoints+2*j+1,0) = features_right[j].Point.y;

        cscmd.Z2.at<double>(scp_general.number_measurements+2*j,0) = features_left[j].Point.x;
        cscmd.Z2.at<double>(scp_general.number_measurements+2*j+1,0) = features_left[j].Point.y;
        cscmd.Z2.at<double>(scp_general.number_measurements+2*NumPoints+2*j,0) = features_right[j].Point.x;
        cscmd.Z2.at<double>(scp_general.number_measurements+2*NumPoints+2*j+1,0) = features_right[j].Point.y;

        cscmd.Z3.at<double>(scp_general.number_measurements+2*j,0) = features_left[j].Point.x;
        cscmd.Z3.at<double>(scp_general.number_measurements+2*j+1,0) = features_left[j].Point.y;
        cscmd.Z3.at<double>(scp_general.number_measurements+2*NumPoints+2*j,0) = features_right[j].Point.x;
        cscmd.Z3.at<double>(scp_general.number_measurements+2*NumPoints+2*j+1,0) = features_right[j].Point.y;

        cscmd.Z4.at<double>(scp_general.number_measurements+2*j,0) = features_left[j].Point.x;
        cscmd.Z4.at<double>(scp_general.number_measurements+2*j+1,0) = features_left[j].Point.y;
        cscmd.Z4.at<double>(scp_general.number_measurements+2*NumPoints+2*j,0) = features_right[j].Point.x;
        cscmd.Z4.at<double>(scp_general.number_measurements+2*NumPoints+2*j+1,0) = features_right[j].Point.y;

        cscmd.R0.at<double>(scp_general.number_measurements+2*j,scp_general.number_measurements+2*j) = features_noise_0*features_noise_0;
        cscmd.R0.at<double>(scp_general.number_measurements+2*j+1,scp_general.number_measurements+2*j+1) = features_noise_0*features_noise_0;
        cscmd.R0.at<double>(scp_general.number_measurements+2*NumPoints+2*j,scp_general.number_measurements+2*NumPoints+2*j) = features_noise_0*features_noise_0;
        cscmd.R0.at<double>(scp_general.number_measurements+2*NumPoints+2*j+1,scp_general.number_measurements+2*NumPoints+2*j+1) = features_noise_0*features_noise_0;

        cscmd.R1.at<double>(scp_general.number_measurements+2*j,scp_general.number_measurements+2*j) = features_noise_1*features_noise_1;
        cscmd.R1.at<double>(scp_general.number_measurements+2*j+1,scp_general.number_measurements+2*j+1) = features_noise_1*features_noise_1;
        cscmd.R1.at<double>(scp_general.number_measurements+2*NumPoints+2*j,scp_general.number_measurements+2*NumPoints+2*j) = features_noise_1*features_noise_1;
        cscmd.R1.at<double>(scp_general.number_measurements+2*NumPoints+2*j+1,scp_general.number_measurements+2*NumPoints+2*j+1) = features_noise_1*features_noise_1;

        cscmd.R2.at<double>(scp_general.number_measurements+2*j,scp_general.number_measurements+2*j) = features_noise_2*features_noise_2;
        cscmd.R2.at<double>(scp_general.number_measurements+2*j+1,scp_general.number_measurements+2*j+1) = features_noise_2*features_noise_2;
        cscmd.R2.at<double>(scp_general.number_measurements+2*NumPoints+2*j,scp_general.number_measurements+2*NumPoints+2*j) = features_noise_2*features_noise_2;
        cscmd.R2.at<double>(scp_general.number_measurements+2*NumPoints+2*j+1,scp_general.number_measurements+2*NumPoints+2*j+1) = features_noise_2*features_noise_2;

        cscmd.R3.at<double>(scp_general.number_measurements+2*j,scp_general.number_measurements+2*j) = features_noise_3*features_noise_3;
        cscmd.R3.at<double>(scp_general.number_measurements+2*j+1,scp_general.number_measurements+2*j+1) = features_noise_3*features_noise_3;
        cscmd.R3.at<double>(scp_general.number_measurements+2*NumPoints+2*j,scp_general.number_measurements+2*NumPoints+2*j) = features_noise_3*features_noise_3;
        cscmd.R3.at<double>(scp_general.number_measurements+2*NumPoints+2*j+1,scp_general.number_measurements+2*NumPoints+2*j+1) = features_noise_3*features_noise_3;

        cscmd.R4.at<double>(scp_general.number_measurements+2*j,scp_general.number_measurements+2*j) = features_noise_4*features_noise_4;
        cscmd.R4.at<double>(scp_general.number_measurements+2*j+1,scp_general.number_measurements+2*j+1) = features_noise_4*features_noise_4;
        cscmd.R4.at<double>(scp_general.number_measurements+2*NumPoints+2*j,scp_general.number_measurements+2*NumPoints+2*j) = features_noise_4*features_noise_4;
        cscmd.R4.at<double>(scp_general.number_measurements+2*NumPoints+2*j+1,scp_general.number_measurements+2*NumPoints+2*j+1) = features_noise_4*features_noise_4;
    }

    /*imshow("il", il);
    imshow("ir", ir);//*/

    //dGF_dZEk+1

    //calculate the dG_dZ only once to speed up the system
    dg_dE=dG_dZ(Range(0,NumPoints),Range(0, scp_general.number_measurements));
    Diff(boost::bind(& calibrationDynamicStereoCameras::G_F, sc0.csc, sc0.get_offsets(), Z_FLKplus1, Z_FRKplus1, _1, _2), Z_EKplus1, dg_dE);

    //dGF_dZFLk+1
    Mat Z_FL_k(2,1,CV_64F);
    Mat Z_FR_k(2,1,CV_64F);
    for(int k=0; k<NumPoints; k++)
    {
        dg_dFL = dG_dZ(Range(k,k+1),Range(scp_general.number_measurements+2*k, scp_general.number_measurements+2*k+1+1));
        dg_dFR = dG_dZ(Range(k,k+1),Range(scp_general.number_measurements+2*NumPoints+2*k, scp_general.number_measurements+2*NumPoints+2*k+1+1));

        Z_FL_k.at<double>(0,0) = Z_FLKplus1.at<double>(2*k,0);
        Z_FL_k.at<double>(1,0) = Z_FLKplus1.at<double>(2*k+1,0);

        Z_FR_k.at<double>(0,0) = Z_FRKplus1.at<double>(2*k,0);
        Z_FR_k.at<double>(1,0) = Z_FRKplus1.at<double>(2*k+1,0);

        Diff(boost::bind(& calibrationDynamicStereoCameras::G_F, sc0.csc, sc0.get_offsets(), _1, Z_FR_k, Z_EKplus1, _2), Z_FL_k, dg_dFL);
        Diff(boost::bind(& calibrationDynamicStereoCameras::G_F, sc0.csc, sc0.get_offsets(), Z_FL_k, _1, Z_EKplus1, _2), Z_FR_k, dg_dFR);
    }

    cscmd.dG_dZ = dG_dZ.clone();
    return cscmd;
}


//give more weight to closer points
double complete_stereo_calib::feature_weight(cv::Point point_left, cv::Point point_right,
cv::Mat Kleft, cv::Mat Kright, cv::Mat T_Unified_2_left, cv::Mat T_Unified_2_right)
{
    cv::Point3d wp = ImageToWorld(point_left, point_right, T_Unified_2_left, T_Unified_2_right,
    Kleft, Kright);

    double x = wp.x;
    double y = wp.y;
    double z = wp.z;

    double w=0;
    double a=500; //1500;
    if(!isinf(x) && !isinf(y) && !isinf(z) && z>0 && !isnan(x) && !isnan(y) && !isnan(z) && z <= a)
    {
        w = 1.;
    }

    return w;
}

//w=0 se o ponto for bom / w=1 se o ponto for mau
//ponto é bom se |z| for igual a |y|
double complete_stereo_calib::feature_weight_rx(cv::Point point, cv::Mat K)
{
    double v = point.y;

    double cy = K.at<double>(1,2);
    double fy = K.at<double>(1,1);

    double mean1 = cy - fy;
    double mean2 = cy + fy;

    double std1 = fy/4.; //fy/4.;
    double std2 = fy/4.; //fy/4.;

    double a1 = -(v-mean1)*(v-mean1)/((2*std1)*(2*std1));
    double a2 = -(v-mean2)*(v-mean2)/((2*std2)*(2*std2));

    double w = exp(a1) + exp(a2);

    return w;
}

//w=0 se o ponto for bom / w=1 se o ponto for mau
//ponto é bom se |z| for igual a |x|
double complete_stereo_calib::feature_weight_ry(cv::Point point, cv::Mat K)
{
    double u = point.x;

    double cx = K.at<double>(0,2);
    double fx = K.at<double>(0,0);

    double mean1 = cx - fx;
    double mean2 = cx + fx;

    double std1 = fx/4.; //fx/4.;
    double std2 = fx/4.; //fx/4.;

    double a1 = -(u-mean1)*(u-mean1)/((2*std1)*(2*std1));
    double a2 = -(u-mean2)*(u-mean2)/((2*std2)*(2*std2));

    double w = exp(a1) + exp(a2);

    return w;
}

//w=0 se o ponto for bom / w=1 se o ponto for mau
//ponto é bom se |z| for igual a |d|, onde d = sqrt(x*x + y*y)
double complete_stereo_calib::feature_weight_rz(cv::Point point, cv::Mat K)
{
    Mat im = Mat::ones(3,1,CV_64F);
    im.at<double>(0,0) = point.x;
    im.at<double>(1,0) = point.y;

    Mat norm_pt = K.inv()*im;

    double xn = norm_pt.at<double>(0,0);
    double yn = norm_pt.at<double>(1,0);

    double d = sqrt(xn*xn + yn*yn);

    double mean1 = 1;
    double std1 = 0.1; //0.1;
    double a = -(d-mean1)*(d-mean1)/((2*std1)*(2*std1));

    double w = exp(a);

    return w;
}

complete_stereo_calib_data complete_stereo_calib::get_calibrated_transformations(cv::Mat cameras_encoders)
{
    complete_stereo_calib_data scd;

    //get filtered system state and covariance matrix
    Mat Offsets = get_offsets();

    Mat calibrated_encoders = Mat::zeros(scp_general.number_measurements, 1,CV_64F);
    calibrated_encoders.at<double>(0,0) = cameras_encoders.at<double>(0,0) - Offsets.at<double>(0,0);
    calibrated_encoders.at<double>(1,0) = cameras_encoders.at<double>(1,0) - Offsets.at<double>(1,0);
    calibrated_encoders.at<double>(2,0) = cameras_encoders.at<double>(2,0) - Offsets.at<double>(2,0);
    calibrated_encoders.at<double>(4,0) = cameras_encoders.at<double>(4,0) - Offsets.at<double>(3,0);
    calibrated_encoders.at<double>(5,0) = cameras_encoders.at<double>(5,0) - Offsets.at<double>(4,0);

    Kinematics_stereoCameras kin;
    KinTransforms KinTr;
    kin.Apply(calibrated_encoders, KinTr);

    scd.transformation_left_cam_to_right_cam = KinTr.LeftCamRefFrame_To_RightCamRefFrame;
    scd.transformation_right_cam_to_left_cam = KinTr.LeftCamRefFrame_To_RightCamRefFrame.inv();
    scd.transformation_left_cam_to_baseline_center = KinTr.UnifiedRefFrame_To_LeftCamRefFrame.inv();
    scd.transformation_right_cam_to_baseline_center = KinTr.UnifiedRefFrame_To_RightCamRefFrame.inv();

    Mat R_left = Mat(3,3,CV_64F);
    Mat R_right = Mat(3,3,CV_64F);
    Mat t_left = Mat(3,1,CV_64F);
    Mat t_right = Mat(3,1,CV_64F);

    for(int r=0; r<3; r++)
    {
        scd.transformation_left_cam_to_right_cam.at<double>(r,3) = scd.transformation_left_cam_to_right_cam.at<double>(r,3)*scp_general.baseline;
        scd.transformation_right_cam_to_left_cam.at<double>(r,3) = scd.transformation_right_cam_to_left_cam.at<double>(r,3)*scp_general.baseline;
        scd.transformation_left_cam_to_baseline_center.at<double>(r,3) = scd.transformation_left_cam_to_baseline_center.at<double>(r,3)*scp_general.baseline;
        scd.transformation_right_cam_to_baseline_center.at<double>(r,3) = scd.transformation_right_cam_to_baseline_center.at<double>(r,3)*scp_general.baseline;

        t_left.at<double>(r,0) = scd.transformation_left_cam_to_right_cam.at<double>(r,3);
        t_right.at<double>(r,0) = scd.transformation_right_cam_to_left_cam.at<double>(r,3);

        for(int c=0; c<3; c++)
        {
            R_left.at<double>(r,c) = scd.transformation_left_cam_to_right_cam.at<double>(r,c);
            R_right.at<double>(r,c) = scd.transformation_right_cam_to_left_cam.at<double>(r,c);
        }
    }

    scd.R_left_cam_to_right_cam = R_left;
    scd.R_right_cam_to_left_cam = R_right;

    scd.t_left_cam_to_right_cam = t_left;
    scd.t_right_cam_to_left_cam = t_right;//*/

    return scd;
}

complete_stereo_disparity_data complete_stereo_calib::get_disparity_map(cv::Mat left_image, cv::Mat right_image, cv::Mat cameras_encoders)
{
    complete_stereo_disparity_data sdd;
    complete_stereo_calib_data scd = get_calibrated_transformations(cameras_encoders);

    cv::Mat R1(3,3,CV_64F), R2(3,3,CV_64F);
    Mat stereo_left_calib_mat, stereo_right_calib_mat;
    Mat Q;

    Mat stereo_rectification_map1_left, stereo_rectification_map2_left, stereo_rectification_map1_right, stereo_rectification_map2_right;
    Mat rectified_left_image, rectified_right_image;

	cv::stereoRectify(Kleft, Mat::zeros(5,1,CV_64F), Kright, Mat::zeros(5,1,CV_64F), Size(left_image.cols, left_image.rows), scd.R_left_cam_to_right_cam, scd.t_left_cam_to_right_cam, R1, R2,
                   stereo_left_calib_mat, stereo_right_calib_mat, Q, CV_CALIB_ZERO_DISPARITY,0, Size(left_image.cols, left_image.rows));

	cv::initUndistortRectifyMap(Kleft, Mat::zeros(5,1,CV_64F), R1, stereo_left_calib_mat, Size(left_image.cols, left_image.rows), CV_16SC2, stereo_rectification_map1_left, stereo_rectification_map2_left);

	cv::initUndistortRectifyMap(Kright, Mat::zeros(5,1,CV_64F), R2, stereo_right_calib_mat, Size(right_image.cols, right_image.rows), CV_16SC2, stereo_rectification_map1_right, stereo_rectification_map2_right);

	cv::remap(left_image, rectified_left_image, stereo_rectification_map1_left, stereo_rectification_map2_left, cv::INTER_LINEAR);
	cv::remap(right_image, rectified_right_image, stereo_rectification_map1_right, stereo_rectification_map2_right, cv::INTER_LINEAR);

	//Disparity Method
	StereoSGBM SBM;

	SBM.numberOfDisparities = 80;
	SBM.preFilterCap = 63;
	SBM.SADWindowSize = 7;
	SBM.P1 = 768;
	SBM.P2 = 1536;
	SBM.minDisparity = 0;
	SBM.uniquenessRatio = 15;
	SBM.speckleWindowSize = 50;
	SBM.speckleRange = 16;
	SBM.disp12MaxDiff = 0;
	SBM.fullDP = true;

	Mat Disparity;
    SBM(rectified_left_image, rectified_right_image, Disparity);
    Disparity.convertTo(sdd.disparity_values, CV_64F, -1/16.);

    //sdd.point_cloud_xyz = Mat::zeros(left_image.rows, left_image.cols, CV_64FC3);
    sdd.disparity_values = Mat::zeros(Disparity.rows,Disparity.cols, CV_64F);

	/*for (int r=0; r<Disparity.rows; r++){
		for (int c=0; c<Disparity.cols; c++){

            sdd.disparity_values.at<double>(r,c) = -double(Disparity.at<short>(r,c))/16;
            //disparity_values.at<double>(r,c) = -double(Disparity.at<short>(r,c))/16;

            Mat imagePoint = Mat::ones(4,1,CV_64F);
            imagePoint.at<double>(0,0) = c;
            imagePoint.at<double>(1,0) = r;
            imagePoint.at<double>(2,0) = -sdd.disparity_values.at<double>(r,c);

            Mat triPoint = Q*imagePoint;
            triPoint = triPoint/triPoint.at<double>(3,0);

	    //triPoint = scd.transformation_left_cam_to_baseline_center*triPoint.clone();

            sdd.point_cloud_xyz.at<Vec3d>(r,c)[0] = triPoint.at<double>(0,0);
            sdd.point_cloud_xyz.at<Vec3d>(r,c)[1] = triPoint.at<double>(1,0);
            sdd.point_cloud_xyz.at<Vec3d>(r,c)[2] = triPoint.at<double>(2,0);

		}
	}//*/
    sdd.point_cloud_rgb = rectified_left_image;
    Disparity.convertTo(sdd.disparity_image, CV_8U, 255/(SBM.numberOfDisparities*16.));

    return sdd;
}
