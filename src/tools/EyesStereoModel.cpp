#include"tools/EyesStereoModel.h"

using namespace std;
using namespace cv;

cv::Mat EyesStereoModel(double ty, double tz, double rx, double ry, double rz)
{
    double h = 480;
    double w = 640;
    double cx = w/2;
    double cy = h/2;
    double fx = 100;
    double fy = 100;
    Mat k = Mat::eye(3,3,CV_64F);
    k.at<double>(0,0) = fx;
    k.at<double>(0,2) = cx;
    k.at<double>(1,1) = fy;
    k.at<double>(1,2) = cy;

    Mat img = Mat::zeros(h,w,CV_8UC3);

    double offset_x_r = -rx;
    double offset_y_l = asin(-tz);
    double offset_y_r = asin(-tz)-ry;
    double offset_z_l = asin(-ty);
    double offset_z_r = asin(-ty)-rz;

    Mat rot_l = Mat::zeros(3,1,CV_64F);
    rot_l.at<double>(1,0) = offset_y_l;
    rot_l.at<double>(2,0) = offset_z_l;

    Mat rot_r = Mat::zeros(3,1,CV_64F);
    rot_r.at<double>(0,0) = offset_x_r;
    rot_r.at<double>(1,0) = offset_y_r;
    rot_r.at<double>(2,0) = offset_z_r;

    Mat R_l, R_r;
    Rodrigues(rot_l, R_l);
    Rodrigues(rot_r, R_r);

    double axis_sz = 50;
    Mat x_axis = Mat::zeros(4,2,CV_64F);
    x_axis.at<double>(0,0) = -axis_sz;
    x_axis.at<double>(0,1) = axis_sz;
    x_axis.at<double>(3,0) = 1;
    x_axis.at<double>(3,1) = 1;

    Mat y_axis = Mat::zeros(4,2,CV_64F);
    y_axis.at<double>(1,0) = -axis_sz;
    y_axis.at<double>(1,1) = axis_sz;
    y_axis.at<double>(3,0) = 1;
    y_axis.at<double>(3,1) = 1;

    Mat z_axis = Mat::zeros(4,2,CV_64F);
    z_axis.at<double>(2,0) = -axis_sz;
    z_axis.at<double>(2,1) = axis_sz;
    z_axis.at<double>(3,0) = 1;
    z_axis.at<double>(3,1) = 1;

    Mat T_l = Mat::eye(4,4,CV_64F);
    Mat T_r = Mat::eye(4,4,CV_64F);
    T_l.at<double>(0,3) = 200;
    T_l.at<double>(1,3) = -50;
    T_l.at<double>(2,3) = -200;

    T_r.at<double>(0,3) = -200;
    T_r.at<double>(1,3) = -50;
    T_r.at<double>(2,3) = -200;

    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            T_l.at<double>(i,j) = R_l.at<double>(i,j);
            T_r.at<double>(i,j) = R_r.at<double>(i,j);
        }
    }

    Mat x_axis_l_rot = T_l*x_axis;
    Mat y_axis_l_rot = T_l*y_axis;
    Mat z_axis_l_rot = T_l*z_axis;

    Mat x_axis_r_rot = T_r*x_axis;
    Mat y_axis_r_rot = T_r*y_axis;
    Mat z_axis_r_rot = T_r*z_axis;

    /*Mat T_l_cam = Mat::eye(4,4,CV_64F);
    Mat T_r_cam = Mat::eye(4,4,CV_64F);
    T_l_cam.at<double>(0,3) = 200;
    T_l_cam.at<double>(1,3) = -200;
    T_l_cam.at<double>(2,3) = -200;

    T_r_cam.at<double>(0,3) = -200;
    T_r_cam.at<double>(1,3) = -200;
    T_r_cam.at<double>(2,3) = -200;

    x_axis_l_rot = T_l_cam*x_axis_l_rot.clone();
    y_axis_l_rot = T_l_cam*y_axis_l_rot.clone();
    z_axis_l_rot = T_l_cam*z_axis_l_rot.clone();

    x_axis_r_rot = T_r_cam*x_axis_r_rot.clone();
    y_axis_r_rot = T_r_cam*y_axis_r_rot.clone();
    z_axis_r_rot = T_r_cam*z_axis_r_rot.clone();//*/

    Point x_axis_l_start, x_axis_l_end, y_axis_l_start, y_axis_l_end, z_axis_l_start, z_axis_l_end;
    Point x_axis_r_start, x_axis_r_end, y_axis_r_start, y_axis_r_end, z_axis_r_start, z_axis_r_end;

    x_axis_l_start.x = round(fx*(x_axis_l_rot.at<double>(0,0)/x_axis_l_rot.at<double>(2,0)) + cx);
    x_axis_l_start.y = round(fy*(x_axis_l_rot.at<double>(1,0)/x_axis_l_rot.at<double>(2,0)) + cy);
    x_axis_l_end.x = round(fx*x_axis_l_rot.at<double>(0,1)/x_axis_l_rot.at<double>(2,1) + cx);
    x_axis_l_end.y = round(fy*x_axis_l_rot.at<double>(1,1)/x_axis_l_rot.at<double>(2,1) + cy);

    y_axis_l_start.x = round(fx*(y_axis_l_rot.at<double>(0,0)/y_axis_l_rot.at<double>(2,0)) + cx);
    y_axis_l_start.y = round(fy*(y_axis_l_rot.at<double>(1,0)/y_axis_l_rot.at<double>(2,0)) + cy);
    y_axis_l_end.x = round(fx*y_axis_l_rot.at<double>(0,1)/y_axis_l_rot.at<double>(2,1) + cx);
    y_axis_l_end.y = round(fy*y_axis_l_rot.at<double>(1,1)/y_axis_l_rot.at<double>(2,1) + cy);

    z_axis_l_start.x = round(fx*(z_axis_l_rot.at<double>(0,0)/z_axis_l_rot.at<double>(2,0)) + cx);
    z_axis_l_start.y = round(fy*(z_axis_l_rot.at<double>(1,0)/z_axis_l_rot.at<double>(2,0)) + cy);
    z_axis_l_end.x = round(fx*z_axis_l_rot.at<double>(0,1)/z_axis_l_rot.at<double>(2,1) + cx);
    z_axis_l_end.y = round(fy*z_axis_l_rot.at<double>(1,1)/z_axis_l_rot.at<double>(2,1) + cy);

    x_axis_r_start.x = round(fx*(x_axis_r_rot.at<double>(0,0)/x_axis_r_rot.at<double>(2,0)) + cx);
    x_axis_r_start.y = round(fy*(x_axis_r_rot.at<double>(1,0)/x_axis_r_rot.at<double>(2,0)) + cy);
    x_axis_r_end.x = round(fx*x_axis_r_rot.at<double>(0,1)/x_axis_r_rot.at<double>(2,1) + cx);
    x_axis_r_end.y = round(fy*x_axis_r_rot.at<double>(1,1)/x_axis_r_rot.at<double>(2,1) + cy);

    y_axis_r_start.x = round(fx*(y_axis_r_rot.at<double>(0,0)/y_axis_r_rot.at<double>(2,0)) + cx);
    y_axis_r_start.y = round(fy*(y_axis_r_rot.at<double>(1,0)/y_axis_r_rot.at<double>(2,0)) + cy);
    y_axis_r_end.x = round(fx*y_axis_r_rot.at<double>(0,1)/y_axis_r_rot.at<double>(2,1) + cx);
    y_axis_r_end.y = round(fy*y_axis_r_rot.at<double>(1,1)/y_axis_r_rot.at<double>(2,1) + cy);

    z_axis_r_start.x = round(fx*(z_axis_r_rot.at<double>(0,0)/z_axis_r_rot.at<double>(2,0)) + cx);
    z_axis_r_start.y = round(fy*(z_axis_r_rot.at<double>(1,0)/z_axis_r_rot.at<double>(2,0)) + cy);
    z_axis_r_end.x = round(fx*z_axis_r_rot.at<double>(0,1)/z_axis_r_rot.at<double>(2,1) + cx);
    z_axis_r_end.y = round(fy*z_axis_r_rot.at<double>(1,1)/z_axis_r_rot.at<double>(2,1) + cy);

    if(x_axis_l_start.x > 0 && x_axis_l_start.x < w && x_axis_l_end.x > 0 && x_axis_l_end.x < w && x_axis_l_start.y > 0 && x_axis_l_start.y < h && x_axis_l_end.y > 0 && x_axis_l_end.y < h &&
       x_axis_r_start.x > 0 && x_axis_r_start.x < w && x_axis_r_end.x > 0 && x_axis_r_end.x < w && x_axis_r_start.y > 0 && x_axis_r_start.y < h && x_axis_r_end.y > 0 && x_axis_r_end.y < h)
    {
        line(img, x_axis_l_start, x_axis_l_end, Scalar(255,0,0),2,8);
        line(img, y_axis_l_start, y_axis_l_end, Scalar(0,255,0),2,8);
        line(img, z_axis_l_start, z_axis_l_end, Scalar(0,0,255),2,8);

        line(img, x_axis_r_start, x_axis_r_end, Scalar(255,0,0),2,8);
        line(img, y_axis_r_start, y_axis_r_end, Scalar(0,255,0),2,8);
        line(img, z_axis_r_start, z_axis_r_end, Scalar(0,0,255),2,8);
    }

    imshow("img", img);

    /*cout << "tt_x_r = " << offset_x_r*180/CV_PI << endl;
    cout << "tt_y_l = " << offset_y_l*180/CV_PI << endl;
    cout << "tt_y_r = " << offset_y_r*180/CV_PI << endl;
    cout << "tt_z_l = " << offset_z_l*180/CV_PI << endl;
    cout << "tt_z_r = " << offset_z_r*180/CV_PI << endl << endl << endl;//*/

    return img;
}
