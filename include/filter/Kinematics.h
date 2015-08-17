#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include "opencv/cv.h"
#include "opencv/highgui.h"

//Kinematics Structure
struct KinTransforms{

	cv::Mat UnifiedRefFrame_To_LeftCamRefFrame;
	cv::Mat UnifiedRefFrame_To_RightCamRefFrame;
	cv::Mat LeftCamRefFrame_To_RightCamRefFrame;
};


//Class for Kinematics
class Kinematics
{
	public:
		virtual void Apply(cv::Mat Encoders, KinTransforms &KinTr)=0;

};

//Class for Kinematics - Chica Head
class Kinematics_stereoCameras: public Kinematics
{
	public:
		void Apply(cv::Mat Encoders, KinTransforms &KinTr);
};

//Rotation Matrix
cv::Mat RotationMatrixX(double rx);
cv::Mat RotationMatrixY(double ry);
cv::Mat RotationMatrixZ(double rz);

#endif
