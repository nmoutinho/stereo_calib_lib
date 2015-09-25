#include "tools/PointsWeight.h"

using namespace cv;

//w=0 se o ponto for bom / w=1 se o ponto for mau
//ponto é bom se |z| for igual a |y|
double PointWeight_rx(cv::Point point, cv::Mat K)
{
    double v = point.y;

    double cy = K.at<double>(1,2);
    double fy = K.at<double>(1,1);

    double mean1 = cy - fy;
    double mean2 = cy + fy;

    double std1 = fy/4.; //fy/4.;
    double std2 = fy/4.; //fy/4.;

    double a1 = -(v-mean1)*(v-mean1)/((2*std1)*(2*std1));
    double a2 = -(v-mean2)*(v-mean2)/((2*std2)*(2*std2));

    double w = exp(a1) + exp(a2);

    return w;
}

//w=0 se o ponto for bom / w=1 se o ponto for mau
//ponto é bom se |z| for igual a |x|
double PointWeight_ry(cv::Point point, cv::Mat K)
{
    double u = point.x;

    double cx = K.at<double>(0,2);
    double fx = K.at<double>(0,0);

    double mean1 = cx - fx;
    double mean2 = cx + fx;

    double std1 = fx/4.; //fx/4.;
    double std2 = fx/4.; //fx/4.;

    double a1 = -(u-mean1)*(u-mean1)/((2*std1)*(2*std1));
    double a2 = -(u-mean2)*(u-mean2)/((2*std2)*(2*std2));

    double w = exp(a1) + exp(a2);

    return w;
}

//w=0 se o ponto for bom / w=1 se o ponto for mau
//ponto é bom se |z| for igual a |d|, onde d = sqrt(x*x + y*y)
double PointWeight_rz(cv::Point point, cv::Mat K)
{
    Mat im = Mat::ones(3,1,CV_64F);
    im.at<double>(0,0) = point.x;
    im.at<double>(1,0) = point.y;

    Mat norm_pt = K.inv()*im;

    double xn = norm_pt.at<double>(0,0);
    double yn = norm_pt.at<double>(1,0);

    double d = sqrt(xn*xn + yn*yn);

    double mean1 = 1;
    double std1 = 0.1; //0.1;
    double a = -(d-mean1)*(d-mean1)/((2*std1)*(2*std1));

    double w = exp(a);

    return w;
}
