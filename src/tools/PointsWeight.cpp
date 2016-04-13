#include "tools/PointsWeight.h"

using namespace cv;
using namespace std;

double PointWeight_ty(cv::Point left_point, cv::Point right_point, cv::Mat Kleft, cv::Mat Kright, cv::Mat T_left_to_right)
{
    cv::Point3d wp = ImageToWorld(left_point, right_point, Mat::eye(4,4,CV_64F), T_left_to_right, Kleft, Kright);

    double x = wp.x;
    double y = wp.y;
    double z = wp.z;

    bool condition = (z<1000);

    double w=0;
    if(!isinf(x) && !isinf(y) && !isinf(z) && z>0 && !isnan(x) && !isnan(y) && !isnan(z))
    {
        if(condition)
            w = 1.;
        else
            w = 0;
    }
    else
        w=1;

    return w;
}

double PointWeight_tz(cv::Point left_point, cv::Point right_point, cv::Mat Kleft, cv::Mat Kright, cv::Mat T_left_to_right)
{
    return PointWeight_ty(left_point, right_point, Kleft, Kright, T_left_to_right);
}

//w=0 se o ponto for bom / w=1 se o ponto for mau
//ponto é bom se |z| for igual a |y|
double PointWeight_rx(cv::Point point, cv::Mat K, int image_height)
{

    double height = double(image_height);

    double v = point.y;

    double w = 0;
    double margin_percent = 0.3;
    if(v <= height*margin_percent || v >= height*(1-margin_percent))
        w=1;

    return w;
}

//w=0 se o ponto for bom / w=1 se o ponto for mau
//ponto é bom se |z| for igual a |x|
double PointWeight_ry(cv::Point point, cv::Mat K, int image_width, int image_height)
{
    double width = double(image_width);
    double height = double(image_height);

    double u = point.x;
    double v = point.y;

    double w = 0;
    double margin_percent_u = 0.3;
    double margin_percent_v = 0.3;
    if((u <= width*margin_percent_u || u >= width*(1-margin_percent_u)) && (v <= height*margin_percent_v || v >= height*(1-margin_percent_v)))
        w=1;

    return w;
}

//w=0 se o ponto for bom / w=1 se o ponto for mau
//ponto é bom se |z| for igual a |d|, onde d = sqrt(x*x + y*y)
double PointWeight_rz(cv::Point point, cv::Mat K, int image_width, int image_height)
{

    double width = double(image_width);
    double height = double(image_height);
    double cx = K.at<double>(0,2);
    double cy = K.at<double>(1,2);

    double u = point.x;
    double v = point.y;

    double w = 0;
    double margin_percent = 0.3;
    if(u <= width*margin_percent || u >= width*(1-margin_percent))
        w=1;

    return w;
}
