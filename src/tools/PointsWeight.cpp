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
        w=0;

    return w;
}

double PointWeight_tz(cv::Point left_point, cv::Point right_point, cv::Mat Kleft, cv::Mat Kright, cv::Mat T_left_to_right)
{
    return PointWeight_ty(left_point, right_point, Kleft, Kright, T_left_to_right);
}

double PointWeight_ty(cv::Point left_point, cv::Point right_point, cv::Mat Kleft, cv::Mat Kright, double baseline, Mat r_rect_left, Mat r_rect_right)
{
    Mat lp = Mat::ones(3,1,CV_64F);
    Mat rp = Mat::ones(3,1,CV_64F);

    lp.at<double>(0,0) = left_point.x;
    lp.at<double>(1,0) = left_point.y;

    rp.at<double>(0,0) = right_point.x;
    rp.at<double>(1,0) = right_point.y;

    Mat lpn = r_rect_left*Kleft.inv()*lp;
    Mat rpn = r_rect_right*Kright.inv()*rp;

    lpn = lpn.clone()/lpn.at<double>(2,0);
    rpn = rpn.clone()/rpn.at<double>(2,0);

    Mat lpi = Kleft*lpn;
    Mat rpi = Kright*rpn;

    double disparity = lpi.at<double>(0,0) - rpi.at<double>(0,0);

    double fx = Kleft.at<double>(0,0);
    double max_z = 1000;

    double min_disparity = fx*baseline/max_z;

    double w = 0.;
    if(disparity >= min_disparity)
        w = 1.;

    return w;
}

double PointWeight_tz(cv::Point left_point, cv::Point right_point, cv::Mat Kleft, cv::Mat Kright, double baseline, Mat r_rect_left, Mat r_rect_right)
{
    Mat lp = Mat::ones(3,1,CV_64F);
    Mat rp = Mat::ones(3,1,CV_64F);

    lp.at<double>(0,0) = left_point.x;
    lp.at<double>(1,0) = left_point.y;

    rp.at<double>(0,0) = right_point.x;
    rp.at<double>(1,0) = right_point.y;

    Mat lpn = r_rect_left*Kleft.inv()*lp;
    Mat rpn = r_rect_right*Kright.inv()*rp;

    lpn = lpn.clone()/lpn.at<double>(2,0);
    rpn = rpn.clone()/rpn.at<double>(2,0);

    Mat lpi = Kleft*lpn;
    Mat rpi = Kright*rpn;

    double disparity = lpi.at<double>(0,0) - rpi.at<double>(0,0);

    double fx = Kleft.at<double>(0,0);
    double max_z = 1000;

    double min_disparity = fx*baseline/max_z;

    double w = 0.;
    if(disparity >= min_disparity)
        w = 1.;

    return w;
}

//w=0 se o ponto for bom / w=1 se o ponto for mau
//ponto é bom se |z| for igual a |y|
double PointWeight_rx(cv::Point point, cv::Mat K, int image_height)
{

    double height = double(image_height);

    double v = point.y;

    /* //double cy = K.at<double>(1,2);
    //double fy = K.at<double>(1,1);

    double mean1 = 0; //cy - fy;
    double mean2 = height-1; //cy + fy;

    double std1 = height/5; //fy/4.; //fy/4.;

    double a1 = -(v-mean1)*(v-mean1)/((2*std1)*(2*std1));
    double a2 = -(v-mean2)*(v-mean2)/((2*std1)*(2*std1));
    double an = -(height-1)*(height-1)/((2*std1)*(2*std1));

    double w = (exp(a1) + exp(a2))/(exp(an)+1);//*/

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

    /*double cx = K.at<double>(0,2);
    double fx = K.at<double>(0,0);

    double mean1 = 0; //cx - fx;
    double mean2 = width-1; //cx + fx;

    double std1 = width/5; //fx/4.; //fx/4.;

    double a1 = -(u-mean1)*(u-mean1)/((2*std1)*(2*std1));
    double a2 = -(u-mean2)*(u-mean2)/((2*std1)*(2*std1));
    double an = -(width-1)*(width-1)/((2*std1)*(2*std1));

    double w = (exp(a1) + exp(a2))/(exp(an)+1);//*/

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

    /*double d = sqrt((u-width/2)*(u-width/2) + (v-height/2)*(v-height/2));
    double mean1 = sqrt((width/2)*(width/2)+(height/2)*(height/2));
    double std1 = width/3;
    double a = -(d-mean1)*(d-mean1)/((2*std1)*(2*std1));

    double w = exp(a);//*/

    double w = 0;
    double margin_percent = 0.3;
    if(u <= width*margin_percent || u >= width*(1-margin_percent))
        w=1;

    return w;
}
