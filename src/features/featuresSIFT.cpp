#include <opencv/cv.h>
#include <vector>
#include "features/featuresSIFT.h"
#include "opencv2/nonfree/features2d.hpp"
#include <iostream>

#pragma warning (disable : 4244) //disable conversion double to float

using namespace std;
using namespace cv;

///Constructor
featuresSIFT::featuresSIFT(void){}

void featuresSIFT::Apply(const cv::Mat &Img1, const cv::Mat &Img2, std::vector<Feature> &Features1, std::vector<Feature> &Features2, int maximumNumberFeatures, double matchingThreshold)
{
    cv::Mat Desc1;
    cv::Mat Desc2;
    std::vector <cv::KeyPoint> kp1, kp2;

    cv::SIFT feature_extractor = SIFT(maximumNumberFeatures);

    feature_extractor(Img1, Mat(), kp1, Desc1);
    feature_extractor(Img2, Mat(), kp2, Desc2);

    std::vector< std::vector< DMatch > > matches;
    std::vector< DMatch > good_matches;
    Ptr<DescriptorMatcher> DM = DescriptorMatcher::create("FlannBased");

    if(kp1.size() > 1 && kp2.size() > 1)
    {
        DM->knnMatch(Desc1, Desc2, matches, 2);
        //std::cout << "after" << std::endl;
        for( int i = 0; i < matches.size(); i++ )
        {
            if(matches[i][0].distance < matchingThreshold*matches[i][1].distance)
            {
                good_matches.push_back( matches[i][0]);
            }
        }

        vector<Point2f> f1_pts, f2_pts;
        for(int i=0; i<good_matches.size(); i++)
        {
            if(i>maximumNumberFeatures)
                break;

            f1_pts.push_back(kp1[good_matches[i].queryIdx].pt);
            f2_pts.push_back(kp2[good_matches[i].trainIdx].pt);
        }

        Mat outliers;
        if(f1_pts.size()>4)//minimum points to findHomography function
        {
            findHomography(f1_pts, f2_pts, CV_RANSAC, 3, outliers);

            Feature f1, f2;
            for(unsigned int i = 0; i<f1_pts.size();i++) {
                if (int(outliers.at<unsigned char>(i,0)))
                {
                    f1.Point = f1_pts[i];
                    f2.Point = f2_pts[i];

                    Features1.push_back(f1);
                    Features2.push_back(f2);
                }
            }
        }
    }
}//*/

