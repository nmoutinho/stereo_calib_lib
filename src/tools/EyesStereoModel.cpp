#include"tools/EyesStereoModel.h"

using namespace std;
using namespace cv;

cv::Mat EyesStereoModel(double ty, double tz, double rx, double ry, double rz)
{
    double h = 300;
    double w = 300;
    double cx = w/2;
    double cy = h/2;
    double fx = h/1.5;
    double fy = fx;
    Mat k = Mat::eye(3,3,CV_64F);
    k.at<double>(0,0) = fx;
    k.at<double>(0,2) = cx;
    k.at<double>(1,1) = fy;
    k.at<double>(1,2) = cy;

    Mat img = Mat::zeros(h,2*w,CV_8UC3);
    Mat l_img = img(Range(0,h), Range(w,2*w));
    Mat r_img = img(Range(0,h), Range(0,w));

    double offset_x_r = -rx;
    double offset_y_l = -asin(tz);
    double offset_y_r = -asin(tz)-ry;
    double offset_z_l = -asin(ty);
    double offset_z_r = -asin(ty)-rz;

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

    double axis_sz = 40;
    Mat axis = Mat::zeros(4,6,CV_64F);
    axis.at<double>(0,0) = -axis_sz;
    axis.at<double>(0,1) = axis_sz;
    axis.at<double>(3,0) = 1;
    axis.at<double>(3,1) = 1;
    axis.at<double>(1,2) = axis_sz;
    axis.at<double>(1,3) = -axis_sz;
    axis.at<double>(3,2) = 1;
    axis.at<double>(3,3) = 1;
    axis.at<double>(2,4) = -axis_sz;
    axis.at<double>(2,5) = axis_sz;
    axis.at<double>(3,4) = 1;
    axis.at<double>(3,5) = 1;

    double optical_axis_sz = 20;
    Mat optical_axis = Mat::zeros(4,1,CV_64F);
    optical_axis.at<double>(2,0) = optical_axis_sz;
    optical_axis.at<double>(3,0) = 1;

    Mat T_l = Mat::eye(4,4,CV_64F);
    Mat T_r = Mat::eye(4,4,CV_64F);
    T_l.at<double>(0,3) = 0;
    T_l.at<double>(1,3) = 0;
    T_l.at<double>(2,3) = -100;

    T_r.at<double>(0,3) = 0;
    T_r.at<double>(1,3) = 0;
    T_r.at<double>(2,3) = -100;

    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            T_l.at<double>(i,j) = R_l.at<double>(i,j);
            T_r.at<double>(i,j) = R_r.at<double>(i,j);
        }
    }

    Mat axis_l_rot = T_l*axis.clone();
    Mat axis_r_rot = T_r*axis.clone();

    Mat optical_axis_l = T_l*optical_axis.clone();
    Mat optical_axis_r = T_r*optical_axis.clone();

    Point x_axis_l_start, x_axis_l_end, y_axis_l_start, y_axis_l_end, z_axis_l_start, z_axis_l_end, optical_axis_l_end;
    Point x_axis_r_start, x_axis_r_end, y_axis_r_start, y_axis_r_end, z_axis_r_start, z_axis_r_end, optical_axis_r_end;

    x_axis_l_start.x = round(fx*(axis_l_rot.at<double>(0,0)/axis_l_rot.at<double>(2,0)) + cx);
    x_axis_l_start.y = round(fy*(axis_l_rot.at<double>(1,0)/axis_l_rot.at<double>(2,0)) + cy);
    x_axis_l_end.x = round(fx*axis_l_rot.at<double>(0,1)/axis_l_rot.at<double>(2,1) + cx);
    x_axis_l_end.y = round(fy*axis_l_rot.at<double>(1,1)/axis_l_rot.at<double>(2,1) + cy);

    y_axis_l_start.x = round(fx*(axis_l_rot.at<double>(0,2)/axis_l_rot.at<double>(2,2)) + cx);
    y_axis_l_start.y = round(fy*(axis_l_rot.at<double>(1,2)/axis_l_rot.at<double>(2,2)) + cy);
    y_axis_l_end.x = round(fx*axis_l_rot.at<double>(0,3)/axis_l_rot.at<double>(2,3) + cx);
    y_axis_l_end.y = round(fy*axis_l_rot.at<double>(1,3)/axis_l_rot.at<double>(2,3) + cy);

    z_axis_l_start.x = round(fx*(axis_l_rot.at<double>(0,4)/axis_l_rot.at<double>(2,4)) + cx);
    z_axis_l_start.y = round(fy*(axis_l_rot.at<double>(1,4)/axis_l_rot.at<double>(2,4)) + cy);
    z_axis_l_end.x = round(fx*axis_l_rot.at<double>(0,5)/axis_l_rot.at<double>(2,5) + cx);
    z_axis_l_end.y = round(fy*axis_l_rot.at<double>(1,5)/axis_l_rot.at<double>(2,5) + cy);

    x_axis_r_start.x = round(fx*(axis_r_rot.at<double>(0,0)/axis_r_rot.at<double>(2,0)) + cx);
    x_axis_r_start.y = round(fy*(axis_r_rot.at<double>(1,0)/axis_r_rot.at<double>(2,0)) + cy);
    x_axis_r_end.x = round(fx*axis_r_rot.at<double>(0,1)/axis_r_rot.at<double>(2,1) + cx);
    x_axis_r_end.y = round(fy*axis_r_rot.at<double>(1,1)/axis_r_rot.at<double>(2,1) + cy);

    y_axis_r_start.x = round(fx*(axis_r_rot.at<double>(0,2)/axis_r_rot.at<double>(2,2)) + cx);
    y_axis_r_start.y = round(fy*(axis_r_rot.at<double>(1,2)/axis_r_rot.at<double>(2,2)) + cy);
    y_axis_r_end.x = round(fx*axis_r_rot.at<double>(0,3)/axis_r_rot.at<double>(2,3) + cx);
    y_axis_r_end.y = round(fy*axis_r_rot.at<double>(1,3)/axis_r_rot.at<double>(2,3) + cy);

    z_axis_r_start.x = round(fx*(axis_r_rot.at<double>(0,4)/axis_r_rot.at<double>(2,4)) + cx);
    z_axis_r_start.y = round(fy*(axis_r_rot.at<double>(1,4)/axis_r_rot.at<double>(2,4)) + cy);
    z_axis_r_end.x = round(fx*axis_r_rot.at<double>(0,5)/axis_r_rot.at<double>(2,5) + cx);
    z_axis_r_end.y = round(fy*axis_r_rot.at<double>(1,5)/axis_r_rot.at<double>(2,5) + cy);

    optical_axis_l_end.x = round(fx*optical_axis_l.at<double>(0,0)/optical_axis_l.at<double>(2,0) + cx);
    optical_axis_l_end.y = round(fy*optical_axis_l.at<double>(1,0)/optical_axis_l.at<double>(2,0) + cy);

    optical_axis_r_end.x = round(fx*optical_axis_r.at<double>(0,0)/optical_axis_r.at<double>(2,0) + cx);
    optical_axis_r_end.y = round(fy*optical_axis_r.at<double>(1,0)/optical_axis_r.at<double>(2,0) + cy);

    //if(x_axis_l_start.x > 0 && x_axis_l_start.x < w && x_axis_l_end.x > 0 && x_axis_l_end.x < w && x_axis_l_start.y > 0 && x_axis_l_start.y < h && x_axis_l_end.y > 0 && x_axis_l_end.y < h &&
    //   x_axis_r_start.x > 0 && x_axis_r_start.x < w && x_axis_r_end.x > 0 && x_axis_r_end.x < w && x_axis_r_start.y > 0 && x_axis_r_start.y < h && x_axis_r_end.y > 0 && x_axis_r_end.y < h)

    {
        //draw cameras
        circle(r_img, Point(w/2,h/2), 50, Scalar(255,255,255), -1);
        circle(r_img, optical_axis_r_end, 20, Scalar(0,0,0), -1);

        circle(l_img, Point(w/2,h/2), 50, Scalar(255,255,255), -1);
        circle(l_img, optical_axis_l_end, 20, Scalar(0,0,0), -1);

        //drawArrow(l_img, x_axis_l_start, x_axis_l_end, Scalar(255,0,0));
        line(l_img, x_axis_l_start, x_axis_l_end, Scalar(255,0,0),1,8);
        line(l_img, y_axis_l_start, y_axis_l_end, Scalar(0,255,0),1,8);
        line(l_img, z_axis_l_start, z_axis_l_end, Scalar(0,0,255),1,8);

        line(r_img, x_axis_r_start, x_axis_r_end, Scalar(255,0,0),1,8);
        line(r_img, y_axis_r_start, y_axis_r_end, Scalar(0,255,0),1,8);
        line(r_img, z_axis_r_start, z_axis_r_end, Scalar(0,0,255),1,8);
    }

    putText(r_img, "rx: "+doubleToString(offset_x_r*180/CV_PI)+"deg", Point(0.25*w,0.1*h), FONT_HERSHEY_TRIPLEX, .4, Scalar(255,255,255));
    putText(r_img, "ry: "+doubleToString(offset_y_r*180/CV_PI)+"deg", Point(0.25*w,0.2*h), FONT_HERSHEY_TRIPLEX, .4, Scalar(255,255,255));
    putText(r_img, "rz: "+doubleToString(offset_z_r*180/CV_PI)+"deg", Point(0.25*w,0.3*h), FONT_HERSHEY_TRIPLEX, .4, Scalar(255,255,255));
    putText(r_img, "Right Camera", Point(0.25*w,0.85*h), FONT_HERSHEY_TRIPLEX, .5, Scalar(255,255,255));

    putText(l_img, "rx: "+doubleToString(0)+"deg", Point(0.25*w,0.1*h), FONT_HERSHEY_TRIPLEX, .4, Scalar(255,255,255));
    putText(l_img, "ry: "+doubleToString(offset_y_l*180/CV_PI)+"deg", Point(0.25*w,0.2*h), FONT_HERSHEY_TRIPLEX, .4, Scalar(255,255,255));
    putText(l_img, "rz: "+doubleToString(offset_z_l*180/CV_PI)+"deg", Point(0.25*w,0.3*h), FONT_HERSHEY_TRIPLEX, .4, Scalar(255,255,255));
    putText(l_img, "Left Camera", Point(0.25*w,0.85*h), FONT_HERSHEY_TRIPLEX, .5, Scalar(255,255,255));

    imshow("img", img);

    /*cout << "tt_x_r = " << offset_x_r*180/CV_PI << endl;
    cout << "tt_y_l = " << offset_y_l*180/CV_PI << endl;
    cout << "tt_y_r = " << offset_y_r*180/CV_PI << endl;
    cout << "tt_z_l = " << offset_z_l*180/CV_PI << endl;
    cout << "tt_z_r = " << offset_z_r*180/CV_PI << endl << endl << endl;//*/

    return l_img;
}

void drawArrow(Mat image, Point p, Point q, Scalar color, int arrowMagnitude, int thickness, int line_type, int shift)
{
    //Draw the principle line
    line(image, p, q, color, thickness, line_type, shift);

    //compute the angle alpha
    double angle = atan2((double)p.y-q.y, (double)p.x-q.x);

    //compute the coordinates of the first segment
    p.x = (int) ( q.x +  arrowMagnitude * cos(angle + CV_PI/4));
    p.y = (int) ( q.y +  arrowMagnitude * sin(angle + CV_PI/4));

    //Draw the first segment
    line(image, p, q, color, thickness, line_type, shift);

    //compute the coordinates of the second segment
    p.x = (int) ( q.x +  arrowMagnitude * cos(angle - CV_PI/4));
    p.y = (int) ( q.y +  arrowMagnitude * sin(angle - CV_PI/4));

    //Draw the second segment
    line(image, p, q, color, thickness, line_type, shift);
}

