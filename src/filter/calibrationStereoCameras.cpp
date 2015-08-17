#include "filter/calibrationStereoCameras.h"
#include "filter/VisualOdometry_Base.h"
#include "filter/Jacobians.h"
#include "filter/EKFBase.h"
#include <iostream>

using namespace cv;

/// Transition State Function F for a Simulated Stereo System
Mat calibrationStereoCameras::F(Mat X_k, Mat U_k) const {

	Mat X_kplus1 = X_k.clone();

	return X_kplus1;
}

///Measurement Function H for a Simulated Stereo System
Mat calibrationStereoCameras::H(Mat X, Mat U) const{

	return Mat();
}


///Measurement Function G for a Simulated Stereo System - Implicit
Mat calibrationStereoCameras::G(Mat X_k, Mat Z) const{

	Mat Inn = Mat::zeros(NumPoints,1,CV_64F);
	//Mat Inn = Mat::zeros(1,1,CV_64F);

	int Num_Fix_Measurements = 0;

	Mat Z_Ekplus1 = Mat::zeros(NumEncodersMeasurements,1,CV_64F);
	if(Flag_Encoders_Measurements){

		for(int j=0 ; j<NumEncodersMeasurements ; j++)
			Z_Ekplus1.at<double>(j,0) = Z.at<double>(Num_Fix_Measurements + j,0);

		Num_Fix_Measurements = Num_Fix_Measurements + NumEncodersMeasurements;
	}

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
	if(Flag_Cameras_Measurements && Flag_Encoders_Measurements)
	{
		calibrationStereoCameras::G_F(X_k, Z_FLkplus1, Z_FRkplus1, Z_Ekplus1, Inn);
	}

	return Inn;

}

//Jacobian dF_dX
cv::Mat calibrationStereoCameras::dF_dX(const cv::Mat &X_k, const cv::Mat &/*U_k*/) const{

	return Mat::eye(X_k.rows, X_k.rows, CV_64F);

}

//Jacobian dG_dZ
cv::Mat calibrationStereoCameras::dG_dZ(const cv::Mat &X_k, const cv::Mat &Z_k) const{

	int Num_Fix_Measurements = 0;

	Mat Z_Ekplus1 = Mat::zeros(NumEncodersMeasurements,1,CV_64F);
	if(Flag_Encoders_Measurements){

		for(int j=0 ; j<NumEncodersMeasurements ; j++)
			Z_Ekplus1.at<double>(j,0) = Z_k.at<double>(Num_Fix_Measurements + j,0);

		Num_Fix_Measurements = Num_Fix_Measurements + NumEncodersMeasurements;
	}

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

	Mat dG_dZ = Mat::zeros(Flag_Encoders_Measurements*Flag_Cameras_Measurements*NumPoints,Z_k.rows,CV_64F);
	//Mat dG_dZ = Mat::zeros(Flag_Encoders_Measurements*Flag_Cameras_Measurements*1,Z_k.rows,CV_64F);
	Mat tmp;


	//dGF_dZ
	if(Flag_Cameras_Measurements && Flag_Encoders_Measurements){

		//dGF_dZEk+1
		tmp=dG_dZ(Range(0,NumPoints),Range(0, NumEncodersMeasurements));
		//tmp=dG_dZ(Range(0,1),Range(0, NumEncodersMeasurements));
		Diff(boost::bind(& calibrationStereoCameras::G_F, this, X_k, Z_FLkplus1, Z_FRkplus1, _1, _2), Z_Ekplus1, tmp);

		//dGF_dZFLk+1
		tmp=dG_dZ(Range(0,NumPoints),Range(NumEncodersMeasurements, NumEncodersMeasurements+2*NumPoints));
		//tmp=dG_dZ(Range(0,1),Range(NumEncodersMeasurements, NumEncodersMeasurements+2*NumPoints));
		Diff(boost::bind(& calibrationStereoCameras::G_F, this, X_k, _1, Z_FRkplus1, Z_Ekplus1, _2), Z_FLkplus1, tmp);

		//dGF_dZFRk+1
		tmp=dG_dZ(Range(0,NumPoints),Range(NumEncodersMeasurements+2*NumPoints, NumEncodersMeasurements+4*NumPoints));
		//tmp=dG_dZ(Range(0,1),Range(NumEncodersMeasurements+2*NumPoints, NumEncodersMeasurements+4*NumPoints));
		Diff(boost::bind(& calibrationStereoCameras::G_F, this, X_k, Z_FLkplus1, _1, Z_Ekplus1, _2), Z_FRkplus1, tmp);
	}

	return dG_dZ;
}

//Sub function for the innovation - Features
void calibrationStereoCameras::G_F(cv::Mat X, cv::Mat Z_FLkplus1, cv::Mat Z_FRkplus1, cv::Mat Z_Ekplus1, cv::Mat &Output) const{

	KinTransforms KinTrK, KinTrKplus1;

	//Kinematics
	Kinematics_stereoCameras Kin;

	Mat RealEncodersKplus1(NumEncodersMeasurements,1,CV_64F);
	RealEncodersKplus1.at<double>(0,0) = Z_Ekplus1.clone().at<double>(0,0)-X.clone().at<double>(0,0);
	RealEncodersKplus1.at<double>(1,0) = Z_Ekplus1.clone().at<double>(1,0)-X.clone().at<double>(1,0);
	RealEncodersKplus1.at<double>(2,0) = Z_Ekplus1.clone().at<double>(2,0)-X.clone().at<double>(2,0);
	RealEncodersKplus1.at<double>(3,0) = Z_Ekplus1.clone().at<double>(3,0);
	RealEncodersKplus1.at<double>(4,0) = Z_Ekplus1.clone().at<double>(4,0)-X.clone().at<double>(3,0);
	RealEncodersKplus1.at<double>(5,0) = Z_Ekplus1.clone().at<double>(5,0)-X.clone().at<double>(4,0);//*/

	//Mat RealEncodersKplus1 = Z_Ekplus1-X;

	Kin.Apply(RealEncodersKplus1,KinTrKplus1);

	Mat T_LeftToRightKplus1 = KinTrKplus1.UnifiedRefFrame_To_RightCamRefFrame.clone()*KinTrKplus1.UnifiedRefFrame_To_LeftCamRefFrame.clone().inv();
	Mat T_RightToLeftKplus1 = T_LeftToRightKplus1.clone().inv();

	Mat R_LeftToRightKplus1 = Mat::zeros(3,3,CV_64F);
	Mat R_RightToLeftKplus1 = Mat::zeros(3,3,CV_64F);

    for(int rrr=0; rrr<3; rrr++)
    {

        for(int ccc=0; ccc<3; ccc++)
        {
            R_LeftToRightKplus1.at<double>(rrr,ccc) = T_LeftToRightKplus1.clone().at<double>(rrr,ccc);
            R_RightToLeftKplus1.at<double>(rrr,ccc) = T_RightToLeftKplus1.clone().at<double>(rrr,ccc);
        }
    }

    double x_lkp1_rkp1 = T_LeftToRightKplus1.clone().at<double>(0,3);
    double y_lkp1_rkp1 = T_LeftToRightKplus1.clone().at<double>(1,3);
    double z_lkp1_rkp1 = T_LeftToRightKplus1.clone().at<double>(2,3);

    double x_rkp1_lkp1 = T_RightToLeftKplus1.clone().at<double>(0,3);
    double y_rkp1_lkp1 = T_RightToLeftKplus1.clone().at<double>(1,3);
    double z_rkp1_lkp1 = T_RightToLeftKplus1.clone().at<double>(2,3);

    Mat Tx_LeftToRightKplus1 = Mat::zeros(3,3,CV_64F);
    Mat Tx_RightToLeftKplus1 = Mat::zeros(3,3,CV_64F);

    Tx_LeftToRightKplus1.at<double>(0,1) = -z_lkp1_rkp1;
    Tx_LeftToRightKplus1.at<double>(1,0) = z_lkp1_rkp1;
    Tx_LeftToRightKplus1.at<double>(0,2) = y_lkp1_rkp1;
    Tx_LeftToRightKplus1.at<double>(2,0) = -y_lkp1_rkp1;
    Tx_LeftToRightKplus1.at<double>(1,2) = -x_lkp1_rkp1;
    Tx_LeftToRightKplus1.at<double>(2,1) = x_lkp1_rkp1;

    Tx_RightToLeftKplus1.at<double>(0,1) = -z_rkp1_lkp1;
    Tx_RightToLeftKplus1.at<double>(1,0) = z_rkp1_lkp1;
    Tx_RightToLeftKplus1.at<double>(0,2) = y_rkp1_lkp1;
    Tx_RightToLeftKplus1.at<double>(2,0) = -y_rkp1_lkp1;
    Tx_RightToLeftKplus1.at<double>(1,2) = -x_rkp1_lkp1;
    Tx_RightToLeftKplus1.at<double>(2,1) = x_rkp1_lkp1;

    Mat E_Lkplus1_Rkplus1 = Tx_LeftToRightKplus1.clone()*R_LeftToRightKplus1.clone();
    Mat F_Lkplus1_Rkplus1 = RightCalibMat.clone().inv().t()*E_Lkplus1_Rkplus1.clone()*LeftCalibMat.clone().inv();

    Mat E_Rkplus1_Lkplus1 = Tx_RightToLeftKplus1.clone()*R_RightToLeftKplus1.clone();
    Mat F_Rkplus1_Lkplus1 = LeftCalibMat.clone().inv().t()*E_Rkplus1_Lkplus1.clone()*RightCalibMat.clone().inv();

    Point2f ImLPtKplus1, ImRPtKplus1;
    double SumErrors = 0;

    for(int i=0; i<NumPoints; i++)
    {
        ImLPtKplus1.x = Z_FLkplus1.clone().at<double>(2*i,0);
        ImLPtKplus1.y = Z_FLkplus1.clone().at<double>(2*i+1,0);
        ImRPtKplus1.x = Z_FRkplus1.clone().at<double>(2*i,0);
        ImRPtKplus1.y = Z_FRkplus1.clone().at<double>(2*i+1,0);

        double epip_lkp12rkp1 = DistanceToEpipolar(ImLPtKplus1, ImRPtKplus1, F_Lkplus1_Rkplus1.clone()); //F_Lkplus1_Rkplus1
        double epip_rkp12lkp1 = DistanceToEpipolar(ImRPtKplus1, ImLPtKplus1, F_Rkplus1_Lkplus1.clone()); //F_Lkplus1_Rkplus1
        //SumErrors = SumErrors + (epip_lkp12rkp1*epip_lkp12rkp1) + (epip_rkp12lkp1*epip_rkp12lkp1);

        Output.at<double>(i,0) = (epip_lkp12rkp1*epip_lkp12rkp1 + epip_rkp12lkp1*epip_rkp12lkp1);
    }
}
