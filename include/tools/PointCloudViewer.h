#ifndef _POINT_CLOUD_VIEWER_H_
#define _POINT_CLOUD_VIEWER_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>

using namespace std;
using namespace cv;

class PointCloudViewer
{

    //image properties
    double image_w;
    double image_h;
    double reference_image_w;
    double reference_image_h;
    double reference_focal_distance;
    double optical_center_x;
    double optical_center_y;
    cv::Mat K;
    cv::Mat image;
    Scalar background_color;

    //camera properties
    double camPos_x;
    double camPos_y;
    double camPos_z;
    double lookAt_x;
    double lookAt_y;
    double lookAt_z;
    double original_camPos_x;
    double original_camPos_y;
    double original_camPos_z;
    double original_lookAt_x;
    double original_lookAt_y;
    double original_lookAt_z;
    double original_pitch;
    double original_yaw;
    double pitch;
    double yaw;
    cv::Mat TransfCam2Orig;
    double x_step;
    double y_step;
    double z_step;
    double yaw_step;
    double pitch_step;

    //debug
    bool debugActive;

    //point cloud
    std::vector<cv::Point3f> pointCloud;
    std::vector<cv::Point3f> pointCloudColor;

    public:
    PointCloudViewer(bool debugActive_=false);

    void set(std::vector<cv::Point3f> pointCloudPoints, std::vector<cv::Point3f> pointCloudRGB);
    void view(string windowName, bool loop = false);
    void setImageProperties(double image_w, double image_h, Scalar background_color_=Scalar(0, 0, 0));

    private:

    void defineCameraParameters(double image_w, double image_h);
    Mat Eular2Rot(double yaw,double pitch, double roll);
    Mat TransformationFromCamToOrigin(double camPos_x, double camPos_y, double camPos_z, double pitch, double yaw);
    void drawAxis(Mat &image, Mat K, Mat TransfCam2Orig, int lineThickness);
    void drawText(Mat &image);
    void createTextLabel(Mat &image, string text, Point textPos, Scalar textColor);

};

class Parallel_PointCloudMapping : public cv::ParallelLoopBody
{
    private:
    std::vector<cv::Point3f> pointCloud;
    std::vector<cv::Point3f> pointCloudColor;
    Mat TransfCam2Orig;
    Mat K;

    public:
    Mat &image;

    Parallel_PointCloudMapping(std::vector<cv::Point3f> pointCloud_, std::vector<cv::Point3f> pointCloudColor_, Mat TransfCam2Orig_, Mat K_,  Mat &image_):
    pointCloud(pointCloud_),
    pointCloudColor(pointCloudColor_),
    TransfCam2Orig(TransfCam2Orig_),
    K(K_),
    image(image_){}

    void operator() (const cv::Range& range) const
    {
         for (int i = range.start; i < range.end; ++i)
         {
            Mat point = Mat::ones(4,1,CV_64F);
            point.at<double>(0,0) = pointCloud[i].x;
            point.at<double>(1,0) = pointCloud[i].y;
            point.at<double>(2,0) = pointCloud[i].z;

            Mat projected_point = TransfCam2Orig*point;

            if(projected_point.at<double>(2,0) > 0)
            {
                Mat norm_point = Mat::ones(3,1,CV_64F);
                norm_point.at<double>(0,0) = projected_point.at<double>(0,0)/projected_point.at<double>(2,0);
                norm_point.at<double>(1,0) = projected_point.at<double>(1,0)/projected_point.at<double>(2,0);

                Mat image_point = K*norm_point;
                circle(image, Point(image_point.at<double>(0,0),image_point.at<double>(1,0)), 2, Scalar(pointCloudColor[i].x,pointCloudColor[i].y,pointCloudColor[i].z), -1);

            }
         }
    }
};

#endif
