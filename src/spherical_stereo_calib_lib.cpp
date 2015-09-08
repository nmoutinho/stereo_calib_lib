#include "spherical_stereo_calib_lib.h"

using namespace std;
using namespace cv;

spherical_stereo_calib::spherical_stereo_calib(spherical_stereo_calib_params sscp_general_)
{
    //the standard image is 320x240
    double resize_factor = 320./sscp_general_.left_cam_resx;

    baseline = sscp_general_.baseline;

    translation_state_noise = 0.33;
    rotation_state_noise = 0.5;
    translation_transition_noise = 0.1;
    rotation_transition_noise = 0.05;
    features_measurements_noise = 5; // /(resize_factor*resize_factor); //5;
    matching_threshold = 0.3;
    max_number_of_features = 200;
    min_number_of_features = 1;
    number_measurements = 6;

    Kleft = Mat::eye(3,3,CV_64F);
    Kleft.at<double>(0,0) = sscp_general_.left_cam_fx;
    Kleft.at<double>(1,1) = sscp_general_.left_cam_fy;
    Kleft.at<double>(0,2) = sscp_general_.left_cam_cx;
    Kleft.at<double>(1,2) = sscp_general_.left_cam_cy;

    Kright = Mat::eye(3,3,CV_64F);
    Kright.at<double>(0,0) = sscp_general_.right_cam_fx;
    Kright.at<double>(1,1) = sscp_general_.right_cam_fy;
    Kright.at<double>(0,2) = sscp_general_.right_cam_cx;
    Kright.at<double>(1,2) = sscp_general_.right_cam_cy;

    number_fixed_state_params = 6;

    csc.Num_Fix_State_Params = number_fixed_state_params;

    //set the filter parameters
    X = Mat::zeros(number_fixed_state_params,1,CV_64F);
    X.at<double>(0,0) = -1; //tx initialized with -1

    P = Mat::eye(number_fixed_state_params,number_fixed_state_params,CV_64F);
    P.at<double>(0,0) = translation_state_noise*translation_state_noise;
    P.at<double>(1,1) = translation_state_noise*translation_state_noise;
    P.at<double>(2,2) = translation_state_noise*translation_state_noise;
    P.at<double>(3,3) = rotation_state_noise*rotation_state_noise;
    P.at<double>(4,4) = rotation_state_noise*rotation_state_noise;
    P.at<double>(5,5) = rotation_state_noise*rotation_state_noise;

    csc.X_k = X.clone();
    csc.P_k = P;
    csc.U_k = Mat::zeros(1,1,CV_64F);
    csc.Pn = Mat::zeros(1,1,CV_64F);
    Mat Q = Mat::eye(number_fixed_state_params,number_fixed_state_params,CV_64F);
    Q.at<double>(0,0) = translation_transition_noise*translation_transition_noise;
    Q.at<double>(1,1) = translation_transition_noise*translation_transition_noise;
    Q.at<double>(2,2) = translation_transition_noise*translation_transition_noise;
    Q.at<double>(3,3) = rotation_transition_noise*rotation_transition_noise;
    Q.at<double>(4,4) = rotation_transition_noise*rotation_transition_noise;
    Q.at<double>(5,5) = rotation_transition_noise*rotation_transition_noise;
    csc.Q = Q;

    //set the camera intrinsic parameters
    csc.LeftCalibMat = Kleft;
    csc.RightCalibMat = Kright;
}

void spherical_stereo_calib::calibrate(const cv::Mat image_left, const cv::Mat image_right)
{
    featuresSIFT get_features;
    std::vector<Feature> features_left;
    std::vector<Feature> features_right;

    get_features.Apply(image_left, image_right, features_left, features_right, max_number_of_features, matching_threshold);

    calibrate(features_left, features_right);
}

void spherical_stereo_calib::calibrate(std::vector<Feature> features_left, std::vector<Feature> features_right)
{
    csc.NumPoints = features_left.size();
    //cout << "Number of features: " << csc.NumPoints << endl;

    if(csc.NumPoints >= min_number_of_features)
    {
        csc.Flag_Cameras_Measurements = true;
    }
    else
    {
        csc.Flag_Cameras_Measurements = false;
    }

    //Prediction
    //boost::posix_time::ptime t0_ = boost::posix_time::microsec_clock::local_time();
    //long t0 = t0_.time_of_day().total_milliseconds();
    csc.Filter_Prediction();

    //Measurements
    //boost::posix_time::ptime t1_ = boost::posix_time::microsec_clock::local_time();
    //long t1 = t1_.time_of_day().total_milliseconds();
    //std::cout << "prediction: " << 1./((t1-t0)/1000.) << "Hz" << std::endl;

    Mat Z(4*csc.NumPoints*csc.Flag_Cameras_Measurements,1,CV_64F);
    Mat Rvec = Mat::zeros(Z.rows, 1,CV_64F);

    //if there are measurements for the cameras
    if(csc.Flag_Cameras_Measurements){

        Mat Z_FLKplus1 = Z(Range(0, 2*csc.NumPoints),Range::all());
        Mat Z_FRKplus1 = Z(Range(2*csc.NumPoints, 4*csc.NumPoints),Range::all());

        for(int j=0; j<csc.NumPoints; j++){

            Z_FLKplus1.at<double>(2*j,0) = features_left[j].Point.x;
            Z_FLKplus1.at<double>(2*j+1,0) = features_left[j].Point.y;

            Z_FRKplus1.at<double>(2*j,0) = features_right[j].Point.x;
            Z_FRKplus1.at<double>(2*j+1,0) = features_right[j].Point.y;
        }

        Mat R_F = Rvec(Range(0, 4*csc.NumPoints), Range::all());
        R_F += features_measurements_noise;

    }//*/

    Mat R = Mat::eye(Rvec.rows,Rvec.rows,CV_64F);
    for(int j=0; j<Rvec.rows; j++)
        R.at<double>(j,j) = Rvec.at<double>(j,0)*Rvec.at<double>(j,0);

    csc.Rn_explicit = Mat();
    csc.Rn_implicit = R.clone();

    //update
    //boost::posix_time::ptime t2_ = boost::posix_time::microsec_clock::local_time();
    //long t2 = t2_.time_of_day().total_milliseconds();
    //std::cout << "measurements: " << 1./((t2-t1)/1000.) << "Hz" << std::endl;

    if(csc.Flag_Cameras_Measurements)
        csc.Filter_Update(Mat(), Z.clone());
}

spherical_stereo_calib_data spherical_stereo_calib::get_calibrated_transformations()
{
    spherical_stereo_calib_data scd;

    Mat X = csc.X_k.clone();

    Mat rot_LeftToRightKplus1 = Mat::zeros(3,1, CV_64F);
	rot_LeftToRightKplus1.at<double>(0,0) = X.clone().at<double>(3,0);
	rot_LeftToRightKplus1.at<double>(1,0) = X.clone().at<double>(4,0);
	rot_LeftToRightKplus1.at<double>(2,0) = X.clone().at<double>(5,0);

	Mat R_LeftToRightKplus1;
	Rodrigues(rot_LeftToRightKplus1, R_LeftToRightKplus1);

	Mat t_LeftToRightKplus1 = Mat::zeros(3,1,CV_64F);
	t_LeftToRightKplus1.at<double>(0,0) = X.at<double>(0,0);
	t_LeftToRightKplus1.at<double>(1,0) = X.at<double>(1,0);
	t_LeftToRightKplus1.at<double>(2,0) = X.at<double>(2,0);

	t_LeftToRightKplus1 = t_LeftToRightKplus1.clone()/norm(t_LeftToRightKplus1.clone())*baseline;

	Mat T_LeftToRightKplus1 = Mat::eye(4,4,CV_64F);
	for (int r=0; r<3; r++)
	{
        T_LeftToRightKplus1.at<double>(r,3) = t_LeftToRightKplus1.at<double>(r,0);

        for (int c=0; c<3; c++)
        {
            T_LeftToRightKplus1.at<double>(r,c) = R_LeftToRightKplus1.at<double>(r,c);
        }
	}

	scd.R_left_cam_to_right_cam = R_LeftToRightKplus1;
	scd.rot_left_cam_to_right_cam = rot_LeftToRightKplus1;
	scd.t_left_cam_to_right_cam = t_LeftToRightKplus1;
	scd.transformation_left_cam_to_right_cam = T_LeftToRightKplus1;

    return scd;
}

spherical_stereo_disparity_data spherical_stereo_calib::get_disparity_map(cv::Mat left_image, cv::Mat right_image)
{
    spherical_stereo_disparity_data sdd;
    spherical_stereo_calib_data scd = get_calibrated_transformations();

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
