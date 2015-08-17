#ifndef _EKF_H_
#define _EKF_H_

#include "opencv/cv.h"
#include "EKFBase.h"

class EKF: public EKFBase {

	public:
		cv::Mat X_k;
		cv::Mat P_k;
		cv::Mat U_k;
		cv::Mat X_pred;
		cv::Mat P_pred;
		cv::Mat Z_explicit_pred;
		cv::Mat Cache_dF_dX;
		cv::Mat Cache_dF_dU;
		cv::Mat Cache_dH_dX;
		cv::Mat Cache_dG_dX;
		cv::Mat Cache_dG_dZ;
		cv::Mat Pn;
		cv::Mat Q;
		cv::Mat Rn_explicit;
		cv::Mat Rn_implicit;
		cv::Mat S;

	public:

		//FILTER PREDICTION
		void Filter_Prediction();

		//FILTER UPDATE
		void Filter_Update(cv::Mat Z_explicit, cv::Mat Z_implicit = cv::Mat());
};

#endif
