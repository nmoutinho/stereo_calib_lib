#include "filter/Rodrigues.h"
#include <iostream>

#define PI 3.14159265359

using namespace cv;

//Sign of X
double sign(double x){
	if(x>0)
		return 1.;
	if(x==0)
		return 0;
	return -1.;
}

///Calculo da matriz de rotacao pela formula de rodrigues
/**	\param rx x angle
	\param ry y angle
	\param rz z angle
	\return 3x3 rotation matrix
	*/
cv::Mat _Rodrigues_Rot(double rx, double ry, double rz){
	//Matriz de rotacao R que retorna
	Mat R(3,3,CV_64F);

	//Teta = ||w*dT||
	double teta=sqrt((rx*rx)+(ry*ry)+(rz*rz));

	Mat U(3,1,CV_64F);

	if (teta<1e-6){
		R.at<double>(0,0)=1;
		R.at<double>(0,1)=-rz;
		R.at<double>(0,2)=ry;
		R.at<double>(1,0)=rz;
		R.at<double>(1,1)=1;
		R.at<double>(1,2)=-rx;
		R.at<double>(2,0)=-ry;
		R.at<double>(2,1)=rx;
		R.at<double>(2,2)=1;
		return R;
	}

	U.at<double>(0,0)=(rx)/teta;
	U.at<double>(1,0)=(ry)/teta;
	U.at<double>(2,0)=(rz)/teta;

	//P = U*U'
	Mat P = U*U.t();

	//Q = ([U]/\)*sin(teta)
	Mat Q(3,3,CV_64F);
	Q.at<double>(0,0)=0;
	Q.at<double>(0,1)=-U.at<double>(2,0);
	Q.at<double>(0,2)=U.at<double>(1,0);
	Q.at<double>(1,0)=U.at<double>(2,0);
	Q.at<double>(1,1)=0;
	Q.at<double>(1,2)=-U.at<double>(0,0);
	Q.at<double>(2,0)=-U.at<double>(1,0);
	Q.at<double>(2,1)=U.at<double>(0,0);
	Q.at<double>(2,2)=0;

	Mat I = Mat::eye(3,3,CV_64F);

	R=P+(I-P)*cos(teta)+Q*sin(teta);

	return R;
}

cv::Mat Inv_Rodrigues_Rot(cv::Mat R){

	Mat W(3,1,CV_64F);
	double trace = (R.at<double>(0,0)+R.at<double>(1,1)+R.at<double>(2,2)-1)/2;

	if(trace >=1){

		W.at<double>(0,0) = 0.5*(R.at<double>(2,1)-R.at<double>(1,2));
		W.at<double>(1,0) = 0.5*(R.at<double>(0,2)-R.at<double>(2,0));
		W.at<double>(2,0) = 0.5*(R.at<double>(1,0)-R.at<double>(0,1));

		return W;

	}

	if(trace <= -1){

		double x2 = (R.at<double>(0,0) + 1)/2;
		double y2 = (R.at<double>(1,1) + 1)/2;
		double z2 = (R.at<double>(2,2) + 1)/2;

		W.at<double>(0,0) = sign(R.at<double>(2,1)-R.at<double>(1,2))*sqrt(x2)*PI;
		W.at<double>(1,0) = sign(R.at<double>(0,2)-R.at<double>(2,0))*sqrt(y2)*PI;
		W.at<double>(2,0) = sign(R.at<double>(1,0)-R.at<double>(0,1))*sqrt(z2)*PI;

		return W;

	}

	double Theta = acos(trace);
	double Aux = 1/(2*sinc(Theta));

	W.at<double>(0,0) = Aux*(R.at<double>(2,1)-R.at<double>(1,2));
	W.at<double>(1,0) = Aux*(R.at<double>(0,2)-R.at<double>(2,0));
	W.at<double>(2,0) = Aux*(R.at<double>(1,0)-R.at<double>(0,1));

	return W;
}

double sinc(double Value){

	if(abs(Value)<1e-6)
		return -Value*Value/6 + 1;


	return sin(Value)/Value;
}
