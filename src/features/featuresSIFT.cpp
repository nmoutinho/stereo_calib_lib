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


/// Acquires a number of NumFeat features in the image Img
/// - the number of features returned could be less or equal to NumFeat
void featuresSIFT::Apply(const cv::Mat &Img1, const cv::Mat &Img2, std::vector<Feature> &Features1, std::vector<Feature> &Features2, int maximumNumberFeatures, double matchingThreshold)
{
    cv::Mat Desc1;
    cv::Mat Desc2;
    std::vector <cv::KeyPoint> kp1, kp2;

    SiftFeatureDetector detector; //(maximumNumberFeatures);
    SiftDescriptorExtractor extractor;

    detector.detect(Img1, kp1);
    detector.detect(Img2, kp2);

    extractor.compute(Img1, kp1, Desc1);
    extractor.compute(Img2, kp2, Desc2);

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

        Feature f1, f2;
        for(int i=0; i<good_matches.size(); i++)
        {
            if(i>maximumNumberFeatures)
                break;

            f1.Point = kp1[good_matches[i].queryIdx].pt;
            f2.Point = kp2[good_matches[i].trainIdx].pt;

            Features1.push_back(f1);
            Features2.push_back(f2);
        }

        Mat img_matches;
        drawMatches( Img1, kp1, Img2, kp2,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        imshow( "Good Matches", img_matches );
        waitKey(10);//*/
    }
}//*/
