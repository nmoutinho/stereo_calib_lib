#ifndef _FEATURES_SIFT_H_
#define _FEATURES_SIFT_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include "featuresBase.h"

class featuresSIFT
{
	public:

		///Constructor
		featuresSIFT(void);

		/// Acquire NumFeat features in the image Img
		void Apply(const cv::Mat &Img1, const cv::Mat &Img2, std::vector<Feature> &Features1, std::vector<Feature> &Features2, int maximumNumberFeatures, double matchingThreshold);

};

#endif
