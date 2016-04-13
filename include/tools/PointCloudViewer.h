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

    //use depth colormap
    bool useDepthColormap;
    double colormapMinDepth;
    double colormapMaxDepth;
    bool insertColormapLegend;

    //point cloud
    std::vector<cv::Point3f> pointCloud;
    std::vector<cv::Point3f> pointCloudColor;

    public:
    PointCloudViewer(bool debugActive_=false);

    void set(std::vector<cv::Point3f> pointCloudPoints, std::vector<cv::Point3f> pointCloudRGB);
    void view(string windowName, bool loop = false);
    void setImageProperties(double image_w, double image_h, Scalar background_color_=Scalar(0, 0, 0));
    void setColormap(double minDepth, double maxDepth, bool insertColormapLegend_=true);

    private:

    Scalar getDepthColormap(double depth, double minDepth, double maxDepth);
    void defineCameraParameters(double image_w, double image_h);
    Mat Eular2Rot(double yaw,double pitch, double roll);
    Mat TransformationFromCamToOrigin(double camPos_x, double camPos_y, double camPos_z, double pitch, double yaw);
    void drawAxis(Mat &image, Mat K, Mat TransfCam2Orig, int lineThickness);
    void drawText(Mat &image);
    void createTextLabel(Mat &image, string text, Point textPos, Scalar textColor);
    void createColormapLegend(Mat &image);
};

#endif
