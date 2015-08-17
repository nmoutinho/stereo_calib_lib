#ifndef _VISUAL_ODOMETRY_STEREO_CAMERAS_IMU_ENCODERS_BASE_H_
#define _VISUAL_ODOMETRY_STEREO_CAMERAS_IMU_ENCODERS_BASE_H_

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "Rodrigues.h"
#include "Kinematics.h"

struct EssentialMatFactorizationData
{
    cv::Mat R;
    cv::Mat t;
};

//Obtain the projection matrix P
cv::Mat ProjectionMat(cv::Mat CamCalibrationMat, cv::Mat TransformationMat);

///Image Features to World Feature
cv::Point3d ImageToWorld(cv::Point2d LeftFeatureCoordinates, cv::Point2d RightFeatureCoordinates, cv::Mat Encoders, cv::Mat LeftCamCalibMat, cv::Mat RightCamCalibMat);
cv::Point3d ImageToWorld(cv::Point2d LeftFeatureCoordinates, cv::Point2d RightFeatureCoordinates,
cv::Mat UnifiedToLeft, cv::Mat UnifiedToRight, cv::Mat LeftCamCalibMat, cv::Mat RightCamCalibMat);

///World Feature to Image Features
void WorldToImage(const cv::Point3d &WorldPoint, cv::Mat Encoders, cv::Mat LeftCamCalibMat, cv::Mat RightCamCalibMat, cv::Point2d &LeftFeatureCoordinates,
				  cv::Point2d &RightFeatureCoordinates);

//Find the minimum distance between two lines
double MinimumLinesDistance(cv::Point2d LeftFeatureCoordinates, cv::Point2d RightFeatureCoordinates, cv::Mat Encoders, cv::Mat LeftCamCalibMat, cv::Mat RightCamCalibMat);

//Remove bad measurements from Zpred and dH_dX
void EliminateMeasurements(int FixedMeasurements, std::vector<int> FeaturesToEliminateIndx, cv::Mat &Zpred, cv::Mat &dH_dX);

//Adjust system state
void AdjustSystemState(int FixedStateParameters, std::vector<int> FeaturesToReplaceIndx, std::vector<cv::Point2f> LeftFeaturesNew, std::vector<cv::Point2f> RightFeaturesNew,
					   double FeatureNoise, cv::Mat &X, cv::Mat &P);


//Obtain the Epipolar Line parameters to find the feature - "EpipolarLineParams = [A B C]^T"
cv::Mat EpipolarLine(const cv::Mat &FundamentalMat, const cv::Mat &ImageFeature);

//Obtain the Distance to the Epipolar line - "EpipolarLineParams = [A B C]^T"
double DistanceToEpipolar(const cv::Mat &K_Left, const cv::Mat &K_Right, const cv::Mat &Tr_Left_To_Right, const cv::Mat &Tr_Right_To_Left,
						  const cv::Mat &Point_Left, const cv::Mat &Point_Right);

double DistanceToEpipolar(cv::Point2f Lpoint, cv::Point2f Rpoint, cv::Mat FundamentalMat);

//Returns a matrix of this form [R | t]
EssentialMatFactorizationData EssentialMatFactorization(std::vector<cv::Point2f> LeftPoints, std::vector<cv::Point2f> RightPoints, cv::Mat Kleft, cv::Mat Kright);

cv::Mat ImagePointToWorldPoint(cv::Point2f ImagePointL, cv::Point2f ImagePointR, cv::Mat Transformation_LtoR, cv::Mat CalibMatL, cv::Mat CalibMatR);

void OpticalFlowAnalysis(std::vector<cv::Point2f> LeftPoints, std::vector<cv::Point2f> RightPoints, cv::Mat Kleft, cv::Mat Kright, int ImgW, int ImgH);

#endif
