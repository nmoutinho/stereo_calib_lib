#include "stereo_calib_lib.h"

using namespace std;
using namespace cv;

void stereo_calib::initialize(stereo_calib_params scp_)
{
    scp = scp_;

    csc.Num_Fix_State_Params = scp.number_fixed_state_params;
    csc.NumEncodersMeasurements = scp.number_measurements;

    //set the filter parameters
    X = Mat::zeros(scp.number_fixed_state_params,1,CV_64F);
    P = Mat::eye(scp.number_fixed_state_params,scp.number_fixed_state_params,CV_64F)*scp.encoders_state_noise*scp.encoders_state_noise;

    csc.X_k = X.clone();
    csc.P_k = P;
    csc.U_k = Mat::zeros(1,1,CV_64F);
    csc.Pn = Mat::zeros(1,1,CV_64F);
    csc.Q = Mat::eye(scp.number_fixed_state_params,scp.number_fixed_state_params,CV_64F)*scp.encoders_transition_noise*scp.encoders_transition_noise;

    //set the camera intrinsic parameters
    Mat left_cam_intrinsic = Mat::eye(3,3,CV_64F);
    left_cam_intrinsic.at<double>(0,0) = scp.left_cam_fx;
    left_cam_intrinsic.at<double>(1,1) = scp.left_cam_fy;
    left_cam_intrinsic.at<double>(0,2) = scp.left_cam_cx;
    left_cam_intrinsic.at<double>(1,2) = scp.left_cam_cy;

    Mat right_cam_intrinsic = Mat::eye(3,3,CV_64F);
    right_cam_intrinsic.at<double>(0,0) = scp.right_cam_fx;
    right_cam_intrinsic.at<double>(1,1) = scp.right_cam_fy;
    right_cam_intrinsic.at<double>(0,2) = scp.right_cam_cx;
    right_cam_intrinsic.at<double>(1,2) = scp.right_cam_cy;

    csc.LeftCalibMat = left_cam_intrinsic;
    csc.RightCalibMat = right_cam_intrinsic;

    csc.calibrate_joint_0 = scp.calibrate_joint_0;
    csc.calibrate_joint_1 = scp.calibrate_joint_1;
    csc.calibrate_joint_2 = scp.calibrate_joint_2;
    csc.calibrate_joint_3 = scp.calibrate_joint_3;
    csc.calibrate_joint_4 = scp.calibrate_joint_4;
    csc.calibrate_joint_5 = scp.calibrate_joint_5;

    starting_flag = true;
}

void stereo_calib::calibrate(std::vector<Feature> features_left, std::vector<Feature> features_right, Mat encoders)
{

    csc.Flag_Cameras_Measurements = false;
    csc.Flag_Encoders_Measurements = false;

    //to start the encoders offsets with the current angles of the cameras
    if(starting_flag)
    {
        int it=0;
        if(scp.calibrate_joint_0)
        {
            X.at<double>(it,0) = encoders.at<double>(0,0);
            it++;
        }
        if(scp.calibrate_joint_1)
        {
            X.at<double>(it,0) = encoders.at<double>(1,0);
            it++;
        }
        if(scp.calibrate_joint_2)
        {
            X.at<double>(it,0) = encoders.at<double>(2,0);
            it++;
        }
        if(scp.calibrate_joint_3)
        {
            X.at<double>(it,0) = encoders.at<double>(3,0);
            it++;
        }
        if(scp.calibrate_joint_4)
        {
            X.at<double>(it,0) = encoders.at<double>(4,0);
            it++;
        }
        if(scp.calibrate_joint_5)
        {
            X.at<double>(it,0) = encoders.at<double>(5,0);
            it++;
        }

        csc.X_k = X.clone();

        starting_flag = false;
    }

    csc.NumPoints = features_left.size();
    //cout << "Number of features: " << csc.NumPoints << endl;

    if(csc.NumPoints >= scp.min_number_of_features)
    {
        csc.Flag_Cameras_Measurements = true;
    }
    else
    {
        csc.Flag_Cameras_Measurements = false;
    }

    csc.Flag_Encoders_Measurements = true;

    //Prediction
    //boost::posix_time::ptime t0_ = boost::posix_time::microsec_clock::local_time();
    //long t0 = t0_.time_of_day().total_milliseconds();
    csc.Filter_Prediction();

    //Measurements
    //boost::posix_time::ptime t1_ = boost::posix_time::microsec_clock::local_time();
    //long t1 = t1_.time_of_day().total_milliseconds();
    //std::cout << "prediction: " << 1./((t1-t0)/1000.) << "Hz" << std::endl;

    Mat Z(csc.NumEncodersMeasurements*csc.Flag_Encoders_Measurements + 4*csc.NumPoints*csc.Flag_Cameras_Measurements,1,CV_64F);
    Mat Rvec = Mat::zeros(Z.rows, 1,CV_64F);

    if(csc.Flag_Encoders_Measurements){

        Mat Z_Ekplus1 = Z(Range(0, csc.NumEncodersMeasurements),Range::all());
        Mat R_Ekplus1 = Rvec(Range(0, csc.NumEncodersMeasurements),Range::all());

        encoders.clone().copyTo(Z_Ekplus1);
        R_Ekplus1 += scp.encoders_measurements_noise;
    }

    //if there are measurements for the cameras
    if(csc.Flag_Cameras_Measurements){

        Mat Z_FLKplus1 = Z(Range(csc.NumEncodersMeasurements, csc.NumEncodersMeasurements+2*csc.NumPoints),Range::all());
        Mat Z_FRKplus1 = Z(Range(csc.NumEncodersMeasurements+2*csc.NumPoints, csc.NumEncodersMeasurements+4*csc.NumPoints),Range::all());

        for(int j=0; j<csc.NumPoints; j++){

            Z_FLKplus1.at<double>(2*j,0) = features_left[j].Point.x;
            Z_FLKplus1.at<double>(2*j+1,0) = features_left[j].Point.y;

            Z_FRKplus1.at<double>(2*j,0) = features_right[j].Point.x;
            Z_FRKplus1.at<double>(2*j+1,0) = features_right[j].Point.y;
        }

        Mat R_F = Rvec(Range(csc.NumEncodersMeasurements, csc.NumEncodersMeasurements+4*csc.NumPoints), Range::all());
        R_F += scp.features_measurements_noise;

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

    if(csc.Flag_Encoders_Measurements && csc.Flag_Cameras_Measurements)
        csc.Filter_Update(Mat(), Z.clone());

    //boost::posix_time::ptime t3_ = boost::posix_time::microsec_clock::local_time();
    //long t3 = t3_.time_of_day().total_milliseconds();
    //std::cout << "update: " << 1./((t3-t2)/1000.) << "Hz" << std::endl;

}

void stereo_calib::calibrate(std::vector<Feature> features_left, std::vector<Feature> features_right, Mat encoders, Mat dG_dZ)
{

    csc.Flag_Cameras_Measurements = false;
    csc.Flag_Encoders_Measurements = false;

    //to start the encoders offsets with the current angles of the cameras
    if(starting_flag)
    {
        int it=0;
        if(scp.calibrate_joint_0)
        {
            X.at<double>(it,0) = encoders.at<double>(0,0);
            it++;
        }
        if(scp.calibrate_joint_1)
        {
            X.at<double>(it,0) = encoders.at<double>(1,0);
            it++;
        }
        if(scp.calibrate_joint_2)
        {
            X.at<double>(it,0) = encoders.at<double>(2,0);
            it++;
        }
        if(scp.calibrate_joint_3)
        {
            X.at<double>(it,0) = encoders.at<double>(3,0);
            it++;
        }
        if(scp.calibrate_joint_4)
        {
            X.at<double>(it,0) = encoders.at<double>(4,0);
            it++;
        }
        if(scp.calibrate_joint_5)
        {
            X.at<double>(it,0) = encoders.at<double>(5,0);
            it++;
        }

        csc.X_k = X.clone();

        starting_flag = false;
    }

    csc.NumPoints = features_left.size();
    //cout << "Number of features: " << csc.NumPoints << endl;

    if(csc.NumPoints >= scp.min_number_of_features)
    {
        csc.Flag_Cameras_Measurements = true;
    }
    else
    {
        csc.Flag_Cameras_Measurements = false;
    }

    csc.Flag_Encoders_Measurements = true;

    //Prediction
    //boost::posix_time::ptime t0_ = boost::posix_time::microsec_clock::local_time();
    //long t0 = t0_.time_of_day().total_milliseconds();
    csc.Filter_Prediction();

    //Measurements
    //boost::posix_time::ptime t1_ = boost::posix_time::microsec_clock::local_time();
    //long t1 = t1_.time_of_day().total_milliseconds();
    //std::cout << "prediction: " << 1./((t1-t0)/1000.) << "Hz" << std::endl;

    Mat Z(csc.NumEncodersMeasurements*csc.Flag_Encoders_Measurements + 4*csc.NumPoints*csc.Flag_Cameras_Measurements,1,CV_64F);
    Mat Rvec = Mat::zeros(Z.rows, 1,CV_64F);

    if(csc.Flag_Encoders_Measurements){

        Mat Z_Ekplus1 = Z(Range(0, csc.NumEncodersMeasurements),Range::all());
        Mat R_Ekplus1 = Rvec(Range(0, csc.NumEncodersMeasurements),Range::all());

        encoders.clone().copyTo(Z_Ekplus1);
        R_Ekplus1 += scp.encoders_measurements_noise;
    }

    //if there are measurements for the cameras
    if(csc.Flag_Cameras_Measurements){

        Mat Z_FLKplus1 = Z(Range(csc.NumEncodersMeasurements, csc.NumEncodersMeasurements+2*csc.NumPoints),Range::all());
        Mat Z_FRKplus1 = Z(Range(csc.NumEncodersMeasurements+2*csc.NumPoints, csc.NumEncodersMeasurements+4*csc.NumPoints),Range::all());

        for(int j=0; j<csc.NumPoints; j++){

            Z_FLKplus1.at<double>(2*j,0) = features_left[j].Point.x;
            Z_FLKplus1.at<double>(2*j+1,0) = features_left[j].Point.y;

            Z_FRKplus1.at<double>(2*j,0) = features_right[j].Point.x;
            Z_FRKplus1.at<double>(2*j+1,0) = features_right[j].Point.y;
        }

        Mat R_F = Rvec(Range(csc.NumEncodersMeasurements, csc.NumEncodersMeasurements+4*csc.NumPoints), Range::all());
        R_F += scp.features_measurements_noise;

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

    csc.Flag_Speed_Up_dG_dZ = true;
    csc.set_dG_dZ(dG_dZ);

    if(csc.Flag_Encoders_Measurements && csc.Flag_Cameras_Measurements)
    {
        csc.Filter_Update(Mat(), Z.clone());
    }

    //boost::posix_time::ptime t3_ = boost::posix_time::microsec_clock::local_time();
    //long t3 = t3_.time_of_day().total_milliseconds();
    //std::cout << "update: " << 1./((t3-t2)/1000.) << "Hz" << std::endl;

}

void stereo_calib::calibrate(cv::Mat Z, cv::Mat R, int NumFeatures, cv::Mat dG_dZ, bool encoders_measurements, bool features_measurements)
{
    csc.Flag_Cameras_Measurements = encoders_measurements;
    csc.Flag_Encoders_Measurements = features_measurements;

    if(starting_flag)
    {
        int it=0;
        if(scp.calibrate_joint_0)
        {
            X.at<double>(it,0) = Z.at<double>(0,0);
            it++;
        }
        if(scp.calibrate_joint_1)
        {
            X.at<double>(it,0) = Z.at<double>(1,0);
            it++;
        }
        if(scp.calibrate_joint_2)
        {
            X.at<double>(it,0) = Z.at<double>(2,0);
            it++;
        }
        if(scp.calibrate_joint_3)
        {
            X.at<double>(it,0) = Z.at<double>(3,0);
            it++;
        }
        if(scp.calibrate_joint_4)
        {
            X.at<double>(it,0) = Z.at<double>(4,0);
            it++;
        }
        if(scp.calibrate_joint_5)
        {
            X.at<double>(it,0) = Z.at<double>(5,0);
            it++;
        }

        csc.X_k = X.clone();

        starting_flag = false;
    }

    csc.Flag_Cameras_Measurements = true;
    csc.Flag_Encoders_Measurements = true;
    csc.NumPoints = NumFeatures;

    //Prediction
    //boost::posix_time::ptime t0_ = boost::posix_time::microsec_clock::local_time();
    //long t0 = t0_.time_of_day().total_milliseconds();
    csc.Filter_Prediction();

    csc.Rn_explicit = Mat();
    csc.Rn_implicit = R.clone();

    csc.Flag_Speed_Up_dG_dZ = true;
    csc.set_dG_dZ(dG_dZ);

    if(csc.Flag_Encoders_Measurements && csc.Flag_Cameras_Measurements)
    {
        csc.Filter_Update(Mat(), Z.clone());
    }

}

Mat stereo_calib::get_offsets()
{
    return csc.X_k.clone();
}

Mat stereo_calib::get_offsets_covariance()
{
    return csc.P_k.clone();
}
