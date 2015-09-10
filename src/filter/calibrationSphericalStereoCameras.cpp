#include "filter/calibrationSphericalStereoCameras.h"
#include "filter/VisualOdometry_Base.h"
#include "filter/Jacobians.h"
#include "filter/EKFBase.h"
#include <iostream>

using namespace cv;

/// Transition State Function F for a Simulated Stereo System
Mat calibrationSphericalStereoCameras::F(Mat X_k, Mat U_k) const {

	Mat X_kplus1 = X_k.clone();

	return X_kplus1;
}

///Measurement Function H for a Simulated Stereo System
Mat calibrationSphericalStereoCameras::H(Mat X, Mat U) const{

	return Mat();
}


///Measurement Function G for a Simulated Stereo System - Implicit
Mat calibrationSphericalStereoCameras::G(Mat X_k, Mat Z) const{

	Mat Inn = Mat::zeros(NumPoints,1,CV_64F);
	//Mat Inn = Mat::zeros(1,1,CV_64F);

	int Num_Fix_Measurements = 0;

	Mat Z_FLkplus1 = Mat::zeros(2*NumPoints,1,CV_64F);
	Mat Z_FRkplus1 = Mat::zeros(2*NumPoints,1,CV_64F);
	if(Flag_Cameras_Measurements){

		for(int i=0; i<NumPoints; i++){

			Z_FLkplus1.at<double>(2*i,0) = Z.at<double>(Num_Fix_Measurements + 2*i,0);
			Z_FLkplus1.at<double>(2*i+1,0) = Z.at<double>(Num_Fix_Measurements + 2*i+1,0);

		}
		Num_Fix_Measurements = Num_Fix_Measurements + 2*NumPoints;

		for(int i=0; i<NumPoints; i++){

			Z_FRkplus1.at<double>(2*i,0) = Z.at<double>(Num_Fix_Measurements + 2*i,0);
			Z_FRkplus1.at<double>(2*i+1,0) = Z.at<double>(Num_Fix_Measurements + 2*i+1,0);

		}
		Num_Fix_Measurements = Num_Fix_Measurements + 2*NumPoints;
	}

	//Innovation for the image points
	if(Flag_Cameras_Measurements)
	{
		calibrationSphericalStereoCameras::G_F(X_k, Z_FLkplus1, Z_FRkplus1, Inn);
	}

	return Inn;

}

//Jacobian dF_dX
cv::Mat calibrationSphericalStereoCameras::dF_dX(const cv::Mat &X_k, const cv::Mat &/*U_k*/) const{

	return Mat::eye(X_k.rows, X_k.rows, CV_64F);

}

//Jacobian dG_dZ
cv::Mat calibrationSphericalStereoCameras::dG_dZ(const cv::Mat &X_k, const cv::Mat &Z_k) const{

	int Num_Fix_Measurements = 0;

	Mat Z_FLkplus1 = Mat::zeros(2*NumPoints,1,CV_64F);
	Mat Z_FRkplus1 = Mat::zeros(2*NumPoints,1,CV_64F);
	if(Flag_Cameras_Measurements){

		for(int i=0; i<NumPoints; i++){

			Z_FLkplus1.at<double>(2*i,0) = Z_k.at<double>(Num_Fix_Measurements + 2*i,0);
			Z_FLkplus1.at<double>(2*i+1,0) = Z_k.at<double>(Num_Fix_Measurements + 2*i+1,0);
		}
		Num_Fix_Measurements = Num_Fix_Measurements + 2*NumPoints;

		for(int i=0; i<NumPoints; i++){

			Z_FRkplus1.at<double>(2*i,0) = Z_k.at<double>(Num_Fix_Measurements + 2*i,0);
			Z_FRkplus1.at<double>(2*i+1,0) = Z_k.at<double>(Num_Fix_Measurements + 2*i+1,0);
		}
		Num_Fix_Measurements = Num_Fix_Measurements + 2*NumPoints;
	}

	Mat dG_dZ = Mat::zeros(Flag_Cameras_Measurements*NumPoints,Z_k.rows,CV_64F);
	//Mat dG_dZ = Mat::zeros(Flag_Encoders_Measurements*Flag_Cameras_Measurements*1,Z_k.rows,CV_64F);
	Mat tmp;

	if(Flag_Cameras_Measurements){

        //dGF_dZFLk+1
        Mat dg_dFL, dg_dFR;
        Mat Z_FL_k(2,1,CV_64F);
        Mat Z_FR_k(2,1,CV_64F);
        for(int k=0; k<NumPoints; k++)
        {
            dg_dFL = dG_dZ(Range(k,k+1),Range(2*k, 2*k+1+1));
            dg_dFR = dG_dZ(Range(k,k+1),Range(2*NumPoints+2*k, 2*NumPoints+2*k+1+1));

            Z_FL_k.at<double>(0,0) = Z_FLkplus1.at<double>(2*k,0);
            Z_FL_k.at<double>(1,0) = Z_FLkplus1.at<double>(2*k+1,0);

            Z_FR_k.at<double>(0,0) = Z_FRkplus1.at<double>(2*k,0);
            Z_FR_k.at<double>(1,0) = Z_FRkplus1.at<double>(2*k+1,0);

            Diff(boost::bind(& calibrationSphericalStereoCameras::G_F, this, X_k, _1, Z_FR_k, _2), Z_FL_k, dg_dFL);
            Diff(boost::bind(& calibrationSphericalStereoCameras::G_F, this, X_k, Z_FL_k, _1, _2), Z_FR_k, dg_dFR);
        }
	}//*/

	return dG_dZ;
}

//Sub function for the innovation - Features
void calibrationSphericalStereoCameras::G_F(cv::Mat X, cv::Mat Z_FLkplus1, cv::Mat Z_FRkplus1, cv::Mat &Output) const{

	Mat rot_LeftToRightKplus1 = Mat::zeros(3,1,  CV_64F);
	rot_LeftToRightKplus1.at<double>(0,0) = X.clone().at<double>(2,0);
	rot_LeftToRightKplus1.at<double>(1,0) = X.clone().at<double>(3,0);
	rot_LeftToRightKplus1.at<double>(2,0) = X.clone().at<double>(4,0);

	Mat R_LeftToRightKplus1;
	Rodrigues(rot_LeftToRightKplus1, R_LeftToRightKplus1);

    double y_lkp1_rkp1 = X.clone().at<double>(0,0);
    double z_lkp1_rkp1 = X.clone().at<double>(1,0);
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

        Output.at<double>(i,0) = (epip_lkp12rkp1*epip_lkp12rkp1 + epip_rkp12lkp1*epip_rkp12lkp1);
        //Output.at<double>(i,0) = (epip_lkp12rkp1*epip_lkp12rkp1);
    }
}
