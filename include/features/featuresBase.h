#ifndef _FEATURES_BASE_H_
#define _FEATURES_BASE_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>

struct Feature{

	//For an image feature
	cv::Point2f Point;
	cv::Mat Descriptor;
	double Quality;

	//For a 3D Feature
	cv::Point3f Position;//x, y, and z
	cv::Point3f Orientation;//theta_x, theta_y and theta_z
	unsigned int FeatureType; //0 - cube; 1 - cylinder; 2 - step; etc...

};
#endif
