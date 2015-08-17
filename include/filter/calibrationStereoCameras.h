#ifndef _CALIBRATION_STEREO_CAMERAS_H_
#define _CALIBRATION_STEREO_CAMERAS_H_

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "Rodrigues.h"
#include "EKF.h"

struct calibrationStereoCameras_data
{
    double matchingThreshold;
    int minNumberFeatures;
    int maxNumberFeatures;
	int numFixStateParams;
	int numMeasurements;
	double encodersStateNoise;
	double encodersTransitionNoise;
	double featuresMeasurementsNoise;
	double encodersMeasurementsNoise;
	int desiredImageWidth;
	int desiredImageHeight;
};

class calibrationStereoCameras: public EKF {

	public:

		double dT;
		int Num_Fix_State_Params;
		int NumPoints;
		cv::Mat LeftCalibMat;
		cv::Mat RightCalibMat;

		int NumEncoders;
		int NumEncodersMeasurements;

		//To select which measurements will be used
		bool Flag_Lin_Acceleration_Measurements;
		bool Flag_Ang_Velocities_Measurements;
		bool Flag_Cameras_Measurements;
		bool Flag_Stereo_Measurements;
		bool Flag_Encoders_Measurements;

	public:

		///Constructor
		calibrationStereoCameras(void){};

		/// Transition State Function F for a Simulated Stereo System
		cv::Mat F(cv::Mat X_k, cv::Mat U_k) const;

		///Measurement Function H for a Simulated Stereo System
		cv::Mat H(cv::Mat X, cv::Mat U) const;

		///Measurement Function G for a Simulated Stereo System - Implicit
		cv::Mat G(cv::Mat X, cv::Mat Z) const;

		//Jacobian dF_dX
		cv::Mat dF_dX(const cv::Mat &X_k, const cv::Mat &U_k) const;

		//Jacobian dG_dZ
		cv::Mat dG_dZ(const cv::Mat &X_k, const cv::Mat &Z_k) const;

	private:

		//Sub function for the innovation - Features
		void G_F(cv::Mat X, cv::Mat Z_FLkplus1, cv::Mat Z_FRkplus1, cv::Mat Z_Ekplus1, cv::Mat &Output) const;


	private:



};

#endif
