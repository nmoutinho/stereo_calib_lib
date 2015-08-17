#ifndef _JACOBIANS_H_
#define _JACOBIANS_H_

#include <opencv/cv.h>
#include <boost/bind.hpp>

///differentiate a functional F at a point X.
template <typename F>
void Diff(const F &f, cv::Mat X, cv::Mat &dF_dX)
{
	double delta = 1e-4; //1e-4;

	cv::Mat Xo = X;

	cv::Mat F_Xo(dF_dX.rows,1,CV_64F);
	f(Xo, F_Xo);

	cv::Mat Delta(X.rows,1,CV_64F);
	cv::Mat Xo_minus_Delta(X.rows,1,CV_64F);

	cv::Mat F_Xo_minus_Delta(dF_dX.rows,1,CV_64F);

	cv::Mat F_Xo_minus_F_Xo_minus_delta(F_Xo.rows,1,CV_64F);

	for (int i=0; i<X.rows; i++){

		Delta = cv::Mat::zeros(X.rows,1,CV_64F);
		Delta.at<double>(i,0)=delta;

		Xo_minus_Delta = Xo-Delta;

		f(Xo_minus_Delta,F_Xo_minus_Delta);

		F_Xo_minus_F_Xo_minus_delta = (F_Xo - F_Xo_minus_Delta)/(delta);

		for (int j=0; j<F_Xo.rows; j++){
			dF_dX.at<double>(j,i)=F_Xo_minus_F_Xo_minus_delta.at<double>(j,0);
		}
	}
}

///differentiate a functional F at a point X.
template <typename F>
cv::Mat Diff(const F &f, cv::Mat X)
{
	double delta = 1e-4; //1e-4;

	cv::Mat Xo = X;

	cv::Mat F_Xo = f(Xo);

	cv::Mat dF_dX(F_Xo.rows,X.rows,CV_64F);

	cv::Mat Delta(X.rows,1,CV_64F);
	cv::Mat Xo_minus_Delta(X.rows,1,CV_64F);

	cv::Mat F_Xo_minus_Delta;

	cv::Mat F_Xo_minus_F_Xo_minus_delta(F_Xo.rows,1,CV_64F);


	for (int i=0; i<X.rows; i++){

		Delta = cv::Mat::zeros(X.rows,1,CV_64F);
		Delta.at<double>(i,0)=delta;

		Xo_minus_Delta = Xo-Delta;

		F_Xo_minus_Delta = f(Xo_minus_Delta);

		F_Xo_minus_F_Xo_minus_delta = (F_Xo - F_Xo_minus_Delta)/(delta);

		for (int j=0; j<F_Xo.rows; j++){
			dF_dX.at<double>(j,i)=F_Xo_minus_F_Xo_minus_delta.at<double>(j,0);
		}
	}
	return dF_dX;//*/
}

#endif
