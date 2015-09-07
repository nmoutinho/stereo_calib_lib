#ifndef _CALIBRATION_SPHERICAL_STEREO_CAMERAS_H_
#define _CALIBRATION_SPHERICAL_STEREO_CAMERAS_H_

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "Rodrigues.h"
#include "EKF.h"

struct calibrationSphericalStereoCameras_data
{
    double matchingThreshold;
    int minNumberFeatures;
    int maxNumberFeatures;
	int numFixStateParams;
	int numMeasurements;
	double translationStateNoise;
	double rotationStateNoise;
	double translationTransitionNoise;
	double rotationTransitionNoise;
	double featuresMeasurementsNoise;
};

class calibrationSphericalStereoCameras: public EKF {

	public:

		double dT;
		int Num_Fix_State_Params;
		int NumPoints;
		cv::Mat LeftCalibMat;
		cv::Mat RightCalibMat;

		//To select which measurements will be used
		bool Flag_Cameras_Measurements;

	public:

		///Constructor
		calibrationSphericalStereoCameras(void){};

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

	public:

		//Sub function for the innovation - Features
		void G_F(cv::Mat X, cv::Mat Z_FLkplus1, cv::Mat Z_FRkplus1, cv::Mat &Output) const;
};

#endif
