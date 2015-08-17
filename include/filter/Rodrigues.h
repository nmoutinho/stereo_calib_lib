#ifndef _RODRIGUES_H_
#define _RODRIGUES_H_

#include "opencv/cv.h"

cv::Mat _Rodrigues_Rot(double rx, double ry, double rz);

cv::Mat Inv_Rodrigues_Rot(cv::Mat R);

double sinc(double Value);

#endif
