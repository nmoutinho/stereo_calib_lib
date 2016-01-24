#include "filter/calibrationSphericalMultipleFilterStereoCameras.h"
#include "filter/VisualOdometry_Base.h"
#include "filter/Jacobians.h"
#include "filter/EKFBase.h"
#include <iostream>

using namespace cv;

calibrationSphericalMultipleFilterStereoCameras::calibrationSphericalMultipleFilterStereoCameras(void)
{
    convergence_threshold = 0.5; //0.025;
    //norm_inn_sq_win = 5;
    mean_inn_samples = 10;
}


/// Transition State Function F for a Simulated Stereo System
Mat calibrationSphericalMultipleFilterStereoCameras::F(Mat X_k, Mat U_k) const {

	Mat X_kplus1 = X_k.clone();

	return X_kplus1;
}

///Measurement Function H for a Simulated Stereo System
Mat calibrationSphericalMultipleFilterStereoCameras::H(Mat X, Mat U) const{

	return Mat();
}


///Measurement Function G for a Simulated Stereo System - Implicit
Mat calibrationSphericalMultipleFilterStereoCameras::G(Mat X_k, Mat Z) const{

	Mat Inn = Mat::zeros(NumPoints,1,CV_64F);
	//Mat Inn = Mat::zeros(1,1,CV_64F);

	int Num_Fix_Measurements = 0;

    Mat Z_Tr_Rot = Mat::zeros(4,1,CV_64F);
	Mat Z_FLkplus1 = Mat::zeros(2*NumPoints,1,CV_64F);
	Mat Z_FRkplus1 = Mat::zeros(2*NumPoints,1,CV_64F);
	if(Flag_Cameras_Measurements){

	    for(int i=0; i<4; i++)
        {
            Z_Tr_Rot.at<double>(i,0) = Z.at<double>(Num_Fix_Measurements + i,0);
        }
        Num_Fix_Measurements = Num_Fix_Measurements + 4;

		for(int i=0; i<NumPoints; i++){

			Z_FLkplus1.at<double>(2*i,0) = Z.at<double>(Num_Fix_Measurements + 4*i,0);
			Z_FLkplus1.at<double>(2*i+1,0) = Z.at<double>(Num_Fix_Measurements + 4*i + 1,0);
			Z_FRkplus1.at<double>(2*i,0) = Z.at<double>(Num_Fix_Measurements + 4*i + 2,0);
			Z_FRkplus1.at<double>(2*i+1,0) = Z.at<double>(Num_Fix_Measurements + 4*i + 3,0);

		}
		//Num_Fix_Measurements = Num_Fix_Measurements + 2*NumPoints;
		Num_Fix_Measurements = Num_Fix_Measurements + 4*NumPoints;

		/*for(int i=0; i<NumPoints; i++){

			Z_FRkplus1.at<double>(2*i,0) = Z.at<double>(Num_Fix_Measurements + 2*i,0);
			Z_FRkplus1.at<double>(2*i+1,0) = Z.at<double>(Num_Fix_Measurements + 2*i+1,0);

		}
		Num_Fix_Measurements = Num_Fix_Measurements + 2*NumPoints;//*/
	}

	//Innovation for the image points
	if(Flag_Cameras_Measurements)
	{
		calibrationSphericalMultipleFilterStereoCameras::G_F(X_k, Z_Tr_Rot, Z_FLkplus1, Z_FRkplus1, Inn);
	}

	return Inn;

}

//Jacobian dF_dX
cv::Mat calibrationSphericalMultipleFilterStereoCameras::dF_dX(const cv::Mat &X_k, const cv::Mat &/*U_k*/) const{

	return Mat::eye(X_k.rows, X_k.rows, CV_64F);

}

//Jacobian dG_dZ
cv::Mat calibrationSphericalMultipleFilterStereoCameras::dG_dZ(const cv::Mat &X_k, const cv::Mat &Z_k) const{

	int Num_Fix_Measurements = 0;

    Mat Z_Tr_Rot = Mat::zeros(4,1,CV_64F);
	Mat Z_FLkplus1 = Mat::zeros(2*NumPoints,1,CV_64F);
	Mat Z_FRkplus1 = Mat::zeros(2*NumPoints,1,CV_64F);
	if(Flag_Cameras_Measurements){

        for(int i=0; i<4; i++)
        {
            Z_Tr_Rot.at<double>(i,0) = Z_k.at<double>(Num_Fix_Measurements + i,0);
        }
        Num_Fix_Measurements = Num_Fix_Measurements + 4;

		for(int i=0; i<NumPoints; i++){

			Z_FLkplus1.at<double>(2*i,0) = Z_k.at<double>(Num_Fix_Measurements + 4*i,0);
			Z_FLkplus1.at<double>(2*i+1,0) = Z_k.at<double>(Num_Fix_Measurements + 4*i + 1,0);
			Z_FRkplus1.at<double>(2*i,0) = Z_k.at<double>(Num_Fix_Measurements + 4*i + 2,0);
			Z_FRkplus1.at<double>(2*i+1,0) = Z_k.at<double>(Num_Fix_Measurements + 4*i + 3,0);
		}
		//Num_Fix_Measurements = Num_Fix_Measurements + 2*NumPoints;
		Num_Fix_Measurements = Num_Fix_Measurements + 4*NumPoints;

		/*for(int i=0; i<NumPoints; i++){

			Z_FRkplus1.at<double>(2*i,0) = Z_k.at<double>(Num_Fix_Measurements + 2*i,0);
			Z_FRkplus1.at<double>(2*i+1,0) = Z_k.at<double>(Num_Fix_Measurements + 2*i+1,0);
		}
		Num_Fix_Measurements = Num_Fix_Measurements + 2*NumPoints;//*/
	}

	Mat dG_dZ = Mat::zeros(Flag_Cameras_Measurements*NumPoints,Z_k.rows,CV_64F);

	if(Flag_Cameras_Measurements){

	    //dGF_dZEk+1
        Mat dg_dTr_Rot = dG_dZ(Range(0,NumPoints),Range(0,4));
        //tmp=dG_dZ(Range(0,1),Range(0, NumEncodersMeasurements));
        Diff(boost::bind(& calibrationSphericalMultipleFilterStereoCameras::G_F, this, X_k, _1, Z_FLkplus1, Z_FRkplus1, _2), Z_Tr_Rot, dg_dTr_Rot);

        //dGF_dZFLk+1
        Mat dg_dFL, dg_dFR;
        Mat Z_FL_k(2,1,CV_64F);
        Mat Z_FR_k(2,1,CV_64F);
        for(int k=0; k<NumPoints; k++)
        {
            dg_dFL = dG_dZ(Range(k,k+1),Range(4 + 2*k, 4 + 2*k+1+1));
            dg_dFR = dG_dZ(Range(k,k+1),Range(4 + 2*NumPoints+2*k, 4 + 2*NumPoints+2*k+1+1));

            Z_FL_k.at<double>(0,0) = Z_FLkplus1.at<double>(2*k,0);
            Z_FL_k.at<double>(1,0) = Z_FLkplus1.at<double>(2*k+1,0);

            Z_FR_k.at<double>(0,0) = Z_FRkplus1.at<double>(2*k,0);
            Z_FR_k.at<double>(1,0) = Z_FRkplus1.at<double>(2*k+1,0);

            Diff(boost::bind(& calibrationSphericalMultipleFilterStereoCameras::G_F, this, X_k, Z_Tr_Rot, _1, Z_FR_k, _2), Z_FL_k, dg_dFL);
            Diff(boost::bind(& calibrationSphericalMultipleFilterStereoCameras::G_F, this, X_k, Z_Tr_Rot, Z_FL_k, _1, _2), Z_FR_k, dg_dFR);
        }
	}//*/

	return dG_dZ;
}

//Sub function for the innovation - Features
void calibrationSphericalMultipleFilterStereoCameras::G_F(cv::Mat X, cv::Mat Z_Tr_Rot, cv::Mat Z_FLkplus1, cv::Mat Z_FRkplus1, cv::Mat &Output) const{


    double ty, tz, rx, ry, rz;

    //if we are estimating ty
    if(id_variable_to_estimate == 0)
    {
        ty = X.clone().at<double>(0,0);

        tz = Z_Tr_Rot.at<double>(0,0);
        rx = Z_Tr_Rot.at<double>(1,0);
        ry = Z_Tr_Rot.at<double>(2,0);
        rz = Z_Tr_Rot.at<double>(3,0);
    }
    //if we are estimating tz
    else if(id_variable_to_estimate == 1)
    {
        tz = X.clone().at<double>(0,0);

        ty = Z_Tr_Rot.at<double>(0,0);
        rx = Z_Tr_Rot.at<double>(1,0);
        ry = Z_Tr_Rot.at<double>(2,0);
        rz = Z_Tr_Rot.at<double>(3,0);
    }
    //if we are estimating rx
    else if(id_variable_to_estimate == 2)
    {
        rx = X.clone().at<double>(0,0);

        ty = Z_Tr_Rot.at<double>(0,0);
        tz = Z_Tr_Rot.at<double>(1,0);
        ry = Z_Tr_Rot.at<double>(2,0);
        rz = Z_Tr_Rot.at<double>(3,0);
    }
    //if we are estimating ry
    else if(id_variable_to_estimate == 3)
    {
        ry = X.clone().at<double>(0,0);

        ty = Z_Tr_Rot.at<double>(0,0);
        tz = Z_Tr_Rot.at<double>(1,0);
        rx = Z_Tr_Rot.at<double>(2,0);
        rz = Z_Tr_Rot.at<double>(3,0);
    }
    //if we are estimating rz
    else if(id_variable_to_estimate == 4)
    {
        rz = X.clone().at<double>(0,0);

        ty = Z_Tr_Rot.at<double>(0,0);
        tz = Z_Tr_Rot.at<double>(1,0);
        rx = Z_Tr_Rot.at<double>(2,0);
        ry = Z_Tr_Rot.at<double>(3,0);
    }

	Mat rot_LeftToRightKplus1 = Mat::zeros(3,1,  CV_64F);
	rot_LeftToRightKplus1.at<double>(0,0) = rx;
	rot_LeftToRightKplus1.at<double>(1,0) = ry;
	rot_LeftToRightKplus1.at<double>(2,0) = rz;

	Mat R_LeftToRightKplus1;
	Rodrigues(rot_LeftToRightKplus1, R_LeftToRightKplus1);

    double y_lkp1_rkp1 = ty;
    double z_lkp1_rkp1 = tz;
    double x_lkp1_rkp1 = -sqrt(1 - y_lkp1_rkp1*y_lkp1_rkp1 - z_lkp1_rkp1*z_lkp1_rkp1);

    Mat Tx_LeftToRightKplus1 = Mat::zeros(3,3,CV_64F);
    //Mat Tx_RightToLeftKplus1 = Mat::zeros(3,3,CV_64F);

    Tx_LeftToRightKplus1.at<double>(0,1) = -z_lkp1_rkp1;
    Tx_LeftToRightKplus1.at<double>(1,0) = z_lkp1_rkp1;
    Tx_LeftToRightKplus1.at<double>(0,2) = y_lkp1_rkp1;
    Tx_LeftToRightKplus1.at<double>(2,0) = -y_lkp1_rkp1;
    Tx_LeftToRightKplus1.at<double>(1,2) = -x_lkp1_rkp1;
    Tx_LeftToRightKplus1.at<double>(2,1) = x_lkp1_rkp1;

    Mat E_Lkplus1_Rkplus1 = Tx_LeftToRightKplus1.clone()*R_LeftToRightKplus1.clone();
    Mat F_Lkplus1_Rkplus1 = RightCalibMat.clone().inv().t()*E_Lkplus1_Rkplus1.clone()*LeftCalibMat.clone().inv();

    Point2f ImLPtKplus1, ImRPtKplus1;
    double SumErrors = 0;

    for(int i=0; i<Output.rows; i++)
    {
        ImLPtKplus1.x = Z_FLkplus1.clone().at<double>(2*i,0);
        ImLPtKplus1.y = Z_FLkplus1.clone().at<double>(2*i+1,0);
        ImRPtKplus1.x = Z_FRkplus1.clone().at<double>(2*i,0);
        ImRPtKplus1.y = Z_FRkplus1.clone().at<double>(2*i+1,0);

        double epip_lkp12rkp1 = DistanceToEpipolar(ImLPtKplus1, ImRPtKplus1, F_Lkplus1_Rkplus1.clone()); //F_Lkplus1_Rkplus1
        double epip_rkp12lkp1 = DistanceToEpipolar(ImRPtKplus1, ImLPtKplus1, F_Lkplus1_Rkplus1.clone().t()); //F_Lkplus1_Rkplus1

        Output.at<double>(i,0) = (epip_lkp12rkp1*epip_lkp12rkp1) + (epip_rkp12lkp1*epip_rkp12lkp1);

        //Output.at<double>(i,0) = (epip_lkp12rkp1*epip_lkp12rkp1);
    }
}
