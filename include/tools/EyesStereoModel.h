#ifndef _EYES_STEREO_MODEL_H_
#define _EYES_STEREO_MODEL_H_

#include <iostream>
#include <string>
#include <sstream>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "tools/ToString.h"
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
cv::Mat EyesStereoModel(double ty, double tz, double rx, double ry, double rz);
void drawArrow(cv::Mat image, cv::Point p, cv::Point q, cv::Scalar color, int arrowMagnitude = 9, int thickness=2, int line_type=8, int shift=0);

#endif
