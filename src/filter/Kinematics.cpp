#include "filter/Kinematics.h"
#include <iostream>

using namespace cv;

//Class for Kinematics - Chica Head
void Kinematics_stereoCameras::Apply(cv::Mat Encoders, KinTransforms &KinTr)
{
	double ry_left_pan      =   Encoders.clone().at<double>(0,0);
	double ry_right_pan     =   Encoders.clone().at<double>(1,0);
	double rx_left_tilt     =   Encoders.clone().at<double>(2,0);
	double rx_right_tilt    =   Encoders.clone().at<double>(3,0);
	double rz_left_swing    =   Encoders.clone().at<double>(4,0);
	double rz_right_swing   =   Encoders.clone().at<double>(5,0);

	double baseline = 0.5;

	double tx_left_center_to_pan = baseline; //Encoders.at<double>(6,0); //34.;
	double tx_right_center_to_pan = -baseline; //Encoders.at<double>(7,0); //-34.;

    //Left Pan
    Mat R_LeftEye_Pan = RotationMatrixY(-ry_left_pan);
	Mat T_LeftEye_Pan = Mat::eye(4,4,CV_64F);
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++){

			T_LeftEye_Pan.at<double>(i,j) = R_LeftEye_Pan.at<double>(i,j);

		}
	}

	//Left Swing
    Mat R_LeftEye_Swing = RotationMatrixZ(-rz_left_swing);
	Mat T_LeftEye_Swing = Mat::eye(4,4,CV_64F);
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++){

			T_LeftEye_Swing.at<double>(i,j) = R_LeftEye_Swing.at<double>(i,j);

		}
	}

	//Left Tilt
    Mat R_LeftEye_Tilt = RotationMatrixX(-rx_left_tilt);
	Mat T_LeftEye_Tilt = Mat::eye(4,4,CV_64F);
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++){

			T_LeftEye_Tilt.at<double>(i,j) = R_LeftEye_Tilt.at<double>(i,j);

		}
	}

	/*//Left Rotation
	Mat T_LeftEye_Rotation = Mat::eye(4,4,CV_64F);
	Mat r_LeftEye = Mat::zeros(3,1,CV_64F);
	r_LeftEye.at<double>(0,0) = -rx_left_tilt;
	r_LeftEye.at<double>(2,0) = -rz_left_swing;
	Mat R_LeftEye;
	Rodrigues(r_LeftEye, R_LeftEye);
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++){

			T_LeftEye_Rotation.at<double>(i,j) = R_LeftEye.clone().at<double>(i,j);

		}
	}//*/

	//Right Pan
    Mat R_RightEye_Pan = RotationMatrixY(-ry_right_pan);
	Mat T_RightEye_Pan = Mat::eye(4,4,CV_64F);
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++){

			T_RightEye_Pan.at<double>(i,j) = R_RightEye_Pan.at<double>(i,j);

		}
	}

	//Right Swing
    Mat R_RightEye_Swing = RotationMatrixZ(-rz_right_swing);
	Mat T_RightEye_Swing = Mat::eye(4,4,CV_64F);
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++){

			T_RightEye_Swing.at<double>(i,j) = R_RightEye_Swing.at<double>(i,j);

		}
	}

	//Right Tilt
    Mat R_RightEye_Tilt = RotationMatrixX(-rx_right_tilt);
	Mat T_RightEye_Tilt = Mat::eye(4,4,CV_64F);
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++){

			T_RightEye_Tilt.at<double>(i,j) = R_RightEye_Tilt.at<double>(i,j);

		}
	}

	/*//Right Rotation
	Mat T_RightEye_Rotation = Mat::eye(4,4,CV_64F);
	Mat r_RightEye = Mat::zeros(3,1,CV_64F);
	r_RightEye.at<double>(0,0) = -rx_right_tilt;
	r_RightEye.at<double>(2,0) = -rz_right_swing;
	Mat R_RightEye;
	Rodrigues(r_RightEye, R_RightEye);
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++){

			T_RightEye_Rotation.at<double>(i,j) = R_RightEye.clone().at<double>(i,j);

		}
	}//*/

	//Baseline - left to right
	Mat T_Baseline_To_Left = Mat::eye(4,4,CV_64F);
	T_Baseline_To_Left.at<double>(0,3) = tx_left_center_to_pan;

	Mat T_Baseline_To_Right = Mat::eye(4,4,CV_64F);
	T_Baseline_To_Right.at<double>(0,3) = tx_right_center_to_pan;

    Mat CenterToLeft = T_LeftEye_Tilt.clone()*T_LeftEye_Swing.clone()*T_LeftEye_Pan.clone()*T_Baseline_To_Left.clone();
    Mat CenterToRight = T_RightEye_Tilt.clone()*T_RightEye_Swing.clone()*T_RightEye_Pan.clone()*T_Baseline_To_Right.clone();

    KinTr.UnifiedRefFrame_To_LeftCamRefFrame = CenterToLeft.clone();
    KinTr.UnifiedRefFrame_To_RightCamRefFrame = CenterToRight.clone();
	KinTr.LeftCamRefFrame_To_RightCamRefFrame = CenterToRight.clone()*CenterToLeft.clone().inv();
}

//Rotation Matrix
cv::Mat RotationMatrixX(double rx)
{
	Mat Rx = Mat::zeros(3,3,CV_64F);

	Rx.at<double>(0,0) = 1;
	Rx.at<double>(1,1) = cos(rx);
	Rx.at<double>(1,2) = -sin(rx);
	Rx.at<double>(2,1) = sin(rx);
	Rx.at<double>(2,2) = cos(rx);

	return Rx;
}

cv::Mat RotationMatrixY(double ry)
{
	Mat Ry = Mat::zeros(3,3,CV_64F);

	Ry.at<double>(0,0) = cos(ry);
	Ry.at<double>(0,2) = sin(ry);
	Ry.at<double>(1,1) = 1;
	Ry.at<double>(2,0) = -sin(ry);
	Ry.at<double>(2,2) = cos(ry);

	return Ry;
}

cv::Mat RotationMatrixZ(double rz)
{
	Mat Rz = Mat::zeros(3,3,CV_64F);

	Rz.at<double>(0,0) = cos(rz);
	Rz.at<double>(0,1) = -sin(rz);
	Rz.at<double>(1,0) = sin(rz);
	Rz.at<double>(1,1) = cos(rz);
	Rz.at<double>(2,2) = 1;

	return Rz;
}
