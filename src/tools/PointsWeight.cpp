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

double PointWeight_ty(double x, double y, double z, double fx, double fy, double baseline, double ty, double min_displacement)
{
    double w=0;
    double total_delta;
    //double min_displacement = 1;
    if(!isinf(x) && !isinf(y) && !isinf(z) && z>0 && !isnan(x) && !isnan(y) && !isnan(z))
    {
        double delta_u = abs(fx*baseline*(sqrt(1-ty*ty)-1)/z);
        double delta_v = abs(fy*baseline*ty/z);
        total_delta = round(delta_u+delta_v);
        if(total_delta >= min_displacement)
        {
            w = 1.;
        }
    }
    else
        w = 1.;

    return w;
}

double PointWeight_tz(cv::Point left_point, cv::Point right_point, cv::Mat Kleft, cv::Mat Kright, cv::Mat T_left_to_right)
{
    cv::Point3d wp = ImageToWorld(left_point, right_point, Mat::eye(4,4,CV_64F), T_left_to_right, Kleft, Kright);

    double x = wp.x;
    double y = wp.y;
    double z = wp.z;

    bool condition = (z<750);

    double w=0;
    if(!isinf(x) && !isinf(y) && !isinf(z) && z>0 && !isnan(x) && !isnan(y) && !isnan(z) && condition)
    {
        w = 1.;
        //std::cout << "Point tz: " << x << " " << y << " " << z << std::endl;
    }

    return w;
}

double PointWeight_tz(double x, double y, double z, double fx, double fy, double baseline, double tz, double min_displacement)
{
    double w=0;
    double total_delta;
    //double min_displacement = 1;
    if(!isinf(x) && !isinf(y) && !isinf(z) && z>0 && !isnan(x) && !isnan(y) && !isnan(z))
    {
        double delta_u = abs(fx*(sqrt(1-tz*tz)*baseline*z-x*tz*baseline-baseline*baseline*tz-baseline*z)/(z*z+tz*baseline*z));
        double delta_v = abs(-fy*y*tz*baseline/(z*z+tz*baseline*z));
        total_delta = round(delta_u+delta_v);
        if(total_delta >= min_displacement)
        {
            w = 1.;
        }
    }
    else
        w = 1.;

    return w;
}

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

double PointWeight_rx(double x, double y, double z, double fx, double fy, double baseline, double rx, double min_displacement)
{
    double w=0;
    double total_delta;
    //double min_displacement = 1;
    if(!isinf(x) && !isinf(y) && !isinf(z) && z>0 && !isnan(x) && !isnan(y) && !isnan(z))
    {
        double delta_u = abs(fx*(z*x+z*baseline-x*z*cos(rx)-x*y*sin(rx)-baseline*z*cos(rx)-baseline*y*sin(rx))/(z*z*cos(rx)+y*z*sin(rx)));
        double delta_v = abs(fy*(z*y*cos(rx)-z*z*sin(rx)-y*z*cos(rx)-y*y*sin(rx))/(z*z*cos(rx)+y*z*sin(rx)));
        total_delta = round(delta_u+delta_v);
        if(total_delta >= min_displacement)
        {
            w = 1.;
        }
    }
    else
        w = 1.;

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

double PointWeight_ry(double x, double y, double z, double fx, double fy, double baseline, double ry, double min_displacement)
{
    double w=0;
    double total_delta;
    //double min_displacement = 1;
    if(!isinf(x) && !isinf(y) && !isinf(z) && z>0 && !isnan(x) && !isnan(y) && !isnan(z))
    {
        double delta_u = abs(fx*(x*x*sin(ry)+z*baseline*cos(ry)+x*baseline*sin(ry)+z*z*sin(ry)-z*baseline)/(z*z*cos(ry)+x*z*sin(ry)));
        double delta_v = abs(fy*(y*z*cos(ry)+y*x*sin(ry)-z*y)/(z*z*cos(ry)+x*z*sin(ry)));
        total_delta = round(delta_u+delta_v);
        if(total_delta >= min_displacement)
        {
            w = 1.;
        }
    }
    else
        w = 1.;

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

    double mean1 = sqrt((width/2)*(width/2)+(height/2)*(height/2));
    double std1 = width/3;
    double a = -(d-mean1)*(d-mean1)/((2*std1)*(2*std1));

    double w = exp(a);

    return w;
}

double PointWeight_rz(double x, double y, double z, double fx, double fy, double baseline, double rz, double min_displacement)
{
    double w=0;
    double total_delta;
    //double min_displacement = 2;
    if(!isinf(x) && !isinf(y) && !isinf(z) && z>0 && !isnan(x) && !isnan(y) && !isnan(z))
    {
        double delta_u = abs(fx*(x-x*cos(rz)+y*sin(rz))/z);
        double delta_v = abs(fy*(y-y*cos(rz)-x*sin(rz))/z);
        total_delta = round(delta_u+delta_v);
        if(total_delta >= min_displacement)
        {
            w = 1.;
        }

    }
    else
        w = 1.;

    return w;
}
