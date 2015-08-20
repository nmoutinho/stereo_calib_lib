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

    scp = scp_general;
    scp.number_fixed_state_params = 5;
    scp.calibrate_joint_0 = true;
    scp.calibrate_joint_1 = true;
    scp.calibrate_joint_2 = true;
    scp.calibrate_joint_3 = false;
    scp.calibrate_joint_4 = true;
    scp.calibrate_joint_5 = true;

    sc.initialize(scp);

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

    Mat offsets_covariance = sc.get_offsets_covariance();

    double min, max;
    minMaxLoc(offsets_covariance, &min, &max);
    double updated_encoders_measurements_noise = sqrt(max);

    double uncertainty = updated_encoders_measurements_noise/scp_general.encoders_state_noise;
    if(uncertainty <= 0.1)
        use_good_points_only = true;

    //cout << "using good points: " << use_good_points_only << endl;
    //cout << "covariance: " << updated_encoders_measurements_noise*180/CV_PI << endl;

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

    if(good_features_left.size() >= scp_general.min_number_of_features)
    {
        sc.calibrate(good_features_left, good_features_right, cameras_encoders);
    }

    offset_0 = sc.get_offsets().at<double>(0,0);
    offset_1 = sc.get_offsets().at<double>(1,0);
    offset_2 = sc.get_offsets().at<double>(2,0);
    offset_3 = sc.get_offsets().at<double>(3,0);
    offset_4 = sc.get_offsets().at<double>(4,0);
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
