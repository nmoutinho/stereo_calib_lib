#ifndef _EYES_STEREO_MODEL_H_
#define _EYES_STEREO_MODEL_H_

#include <iostream>
#include <string>
#include <sstream>
#include "opencv/cv.h"
#include "opencv/highgui.h"

cv::Mat EyesStereoModel(double ty, double tz, double rx, double ry, double rz);

#endif
