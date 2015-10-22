#ifndef _EKF_BASE_H_
#define _EKF_BASE_H_

#include "opencv/cv.h"

class EKFBase{
	public:

		virtual cv::Mat F(cv::Mat, cv::Mat) const=0;
		virtual cv::Mat H(cv::Mat, cv::Mat) const=0;
		virtual cv::Mat G(cv::Mat, cv::Mat) const=0;
		virtual cv::Mat dF_dX(const cv::Mat &, const cv::Mat &) const;
		virtual cv::Mat dF_dU(const cv::Mat &, const cv::Mat &) const;
		virtual cv::Mat dH_dX(const cv::Mat &, const cv::Mat &) const;
		virtual cv::Mat dG_dX(const cv::Mat &, const cv::Mat &) const;
		virtual cv::Mat dG_dZ(const cv::Mat &, const cv::Mat &) const;

		std::vector<double> norm_inn_sq;
		bool filter_converged;
		double convergence_threshold;
		int norm_inn_sq_win;
};

//Implementation of the Prediction Function
void Prediction(cv::Mat X_k, cv::Mat U_k, cv::Mat P_k, cv::Mat Pn, cv::Mat Q, cv::Mat R_explicit, const EKFBase &EKFb, cv::Mat &X_pred,
				     cv::Mat &P_pred, cv::Mat &Z_explicit_pred, cv::Mat &dF_dX, cv::Mat &dH_dX, cv::Mat &dF_dU, cv::Mat &S);

//Implementation of the Update Function
void Explicit_Update(cv::Mat X_pred, cv::Mat P_pred, cv::Mat Z_explicit, cv::Mat Z_explicit_pred, cv::Mat Pn, cv::Mat Q, cv::Mat R_explicit, cv::Mat dF_dX,
			 cv::Mat dH_dX, cv::Mat dF_dU, cv::Mat &X_kplus1, cv::Mat &P_kplus1);

void Innovation_Update(cv::Mat X_pred, cv::Mat P_pred, cv::Mat Inn, cv::Mat Pn, cv::Mat Q, cv::Mat R, cv::Mat dF_dX,
			 cv::Mat dInn_dX, cv::Mat dF_dU, cv::Mat &X_kplus1, cv::Mat &P_kplus1);

void Innovation_Update(cv::Mat X_pred, cv::Mat P_pred, cv::Mat Inn, cv::Mat Pn, cv::Mat Q, cv::Mat R, cv::Mat dF_dX, cv::Mat dInn_dX, cv::Mat dF_dU,
			 cv::Mat &X_kplus1, cv::Mat &P_kplus1, std::vector<double> &norm_inn_sq, bool &filter_converged, int norm_inn_sq_win, double convergence_threshold);

void Implicit_Explicit_Update(cv::Mat X_pred, cv::Mat P_pred, cv::Mat Z_explicit, cv::Mat Z_explicit_pred, cv::Mat Inn_implicit, cv::Mat Pn, cv::Mat Q, cv::Mat R_explicit,
                              cv::Mat R_implicit, cv::Mat dF_dX, cv::Mat dH_dX, cv::Mat dG_dX, cv::Mat dF_dU, cv::Mat &X_kplus1, cv::Mat &P_kplus1);


void Implicit_Explicit_Update(cv::Mat X_pred, cv::Mat P_pred, cv::Mat Z_explicit, cv::Mat Z_explicit_pred, cv::Mat Inn_implicit, cv::Mat Pn, cv::Mat Q, cv::Mat R_explicit,
							  cv::Mat R_implicit, cv::Mat dF_dX, cv::Mat dH_dX, cv::Mat dG_dX, cv::Mat dF_dU, cv::Mat &X_kplus1, cv::Mat &P_kplus1,
							  std::vector<double> &norm_inn_sq, bool &filter_converged, int norm_inn_sq_win, double convergence_threshold);

#endif
