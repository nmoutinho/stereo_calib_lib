#include "tools/PointsWeight.h"

using namespace cv;

//w=0 se o ponto for bom / w=1 se o ponto for mau
//ponto é bom se |z| for igual a |y|
double PointWeight_rx(cv::Point point, cv::Mat K, int image_height)
{

    double height = double(image_height);

    double v = point.y;

    double cy = K.at<double>(1,2);
    double fy = K.at<double>(1,1);

    double mean1 = 0; //cy - fy;
    double mean2 = height-1; //cy + fy;

    double std1 = height/5; //fy/4.; //fy/4.;

    double a1 = -(v-mean1)*(v-mean1)/((2*std1)*(2*std1));
    double a2 = -(v-mean2)*(v-mean2)/((2*std1)*(2*std1));
    double an = -(height-1)*(height-1)/((2*std1)*(2*std1));

    double w = (exp(a1) + exp(a2))/(exp(an)+1);
    return w;
}

//w=0 se o ponto for bom / w=1 se o ponto for mau
//ponto é bom se |z| for igual a |x|
double PointWeight_ry(cv::Point point, cv::Mat K, int image_width)
{
    double width = double(image_width);

    double u = point.x;

    double cx = K.at<double>(0,2);
    double fx = K.at<double>(0,0);

    double mean1 = 0; //cx - fx;
    double mean2 = width-1; //cx + fx;

    double std1 = width/5; //fx/4.; //fx/4.;

    double a1 = -(u-mean1)*(u-mean1)/((2*std1)*(2*std1));
    double a2 = -(u-mean2)*(u-mean2)/((2*std1)*(2*std1));
    double an = -(width-1)*(width-1)/((2*std1)*(2*std1));

    double w = (exp(a1) + exp(a2))/(exp(an)+1);
    return w;
}

//w=0 se o ponto for bom / w=1 se o ponto for mau
//ponto é bom se |z| for igual a |d|, onde d = sqrt(x*x + y*y)
double PointWeight_rz(cv::Point point, cv::Mat K, int image_width, int image_height)
{

    double width = double(image_width);
    double height = double(image_height);

    double u = point.x;
    double v = point.y;

    double d = sqrt((u-width/2)*(u-width/2) + (v-height/2)*(v-height/2));

    double mean1 = width/2;
    double std1 = width/5;
    double a = -(d-mean1)*(d-mean1)/((2*std1)*(2*std1));

    double w = exp(a);

    return w;
}
