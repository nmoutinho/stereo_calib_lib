#ifndef _FEATURES_SIMULATED_H_
#define _FEATURES_SIMULATED_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include "featuresBase.h"

class featuresSIMULATED
{
	public:

		///Constructor
		featuresSIMULATED(void);

		/// Acquire NumFeat features in the image Img
		void Apply(std::vector<Feature> &Features1, std::vector<Feature> &Features2,
		cv::Mat kleft, cv::Mat kright, int image_w, int image_h, cv::Mat T_1to2, int numberFeatures, bool further_points=false);

};

#endif
