#include <iostream>

#include "filter/EKFBase.h"
#include "filter/Jacobians.h"

using namespace cv;
using namespace std;

cv::Mat EKFBase::dF_dX(const cv::Mat &X_k, const cv::Mat &U_k) const{

	return Diff(boost::bind(& EKFBase::F, this, _1, U_k), X_k);

}

cv::Mat EKFBase::dF_dU(const cv::Mat &X_k, const cv::Mat &U_k) const{

	return Diff(boost::bind(& EKFBase::F, this, X_k, _1), U_k);

}

cv::Mat EKFBase::dH_dX(const cv::Mat &X_k, const cv::Mat &Z_k) const{

	return Diff(boost::bind(& EKFBase::H, this, _1, Z_k), X_k);

}

cv::Mat EKFBase::dG_dX(const cv::Mat &X_k, const cv::Mat &Z_k) const{

	return Diff(boost::bind(& EKFBase::G, this, _1, Z_k), X_k);

}

cv::Mat EKFBase::dG_dZ(const cv::Mat &X_k, const cv::Mat &Z_k) const{

	return Diff(boost::bind(& EKFBase::G, this, X_k, _1), Z_k);

}

//Implementation of the Prediction Function
void Prediction(Mat X_k, Mat U_k, Mat P_k, Mat Pn, Mat Q, Mat R_explicit, const EKFBase &EKFb, Mat &X_pred, Mat &P_pred, Mat &Z_explicit_pred, Mat &dF_dX, Mat &dH_dX,
		  Mat &dF_dU, Mat &S){

		//Predicao do estado do sistema
		X_pred = EKFb.F(X_k, U_k);

		//Jacobianas
		dF_dX = EKFb.dF_dX(X_pred, U_k);

		dH_dX = EKFb.dH_dX(X_pred, U_k);;

		dF_dU = EKFb.dF_dU(X_pred, U_k);

		//Predicao da matriz de covariancia do sistema
		Mat W = (dF_dU*Pn*dF_dU.t()); //Matriz auxiliar
		P_pred = (dF_dX*P_k*dF_dX.t())+W+Q;

		//Predicao das medicoes
		Z_explicit_pred = EKFb.H(X_pred, U_k);

		//Matriz covariancia do processo de inovacao
		S = (dH_dX * P_pred * dH_dX.t()) + R_explicit;
}


//Implementation of the Update Function
void Explicit_Update(Mat X_pred, Mat P_pred, Mat Z_explicit, Mat Z_explicit_pred, Mat Pn, Mat Q, Mat R_explicit, Mat dF_dX, Mat dH_dX, Mat dF_dU,
			 Mat &X_kplus1, Mat &P_kplus1){

	//Processo inovacao v
	Mat Inn = Z_explicit_pred.clone() - Z_explicit.clone();

	//cout << "Innovation: " << Inn << endl;

	Mat R = R_explicit.clone();

	Innovation_Update(X_pred.clone(), P_pred.clone(), Inn.clone(), Pn.clone(), Q.clone(), R.clone(), dF_dX.clone(), dH_dX.clone(), dF_dU.clone(), X_kplus1, P_kplus1);

}


void Implicit_Explicit_Update(Mat X_pred, Mat P_pred, Mat Z_explicit, Mat Z_explicit_pred, Mat Inn_implicit, Mat Pn, Mat Q, Mat R_explicit, Mat R_implicit, Mat dF_dX,
							  Mat dH_dX, Mat dG_dX, Mat dF_dU, Mat &X_kplus1, Mat &P_kplus1){

	//Case when Explicit Measurements are empty
	if(Inn_implicit.rows>0 && Z_explicit.rows == 0){

		Mat R = R_implicit.clone();

		Innovation_Update(X_pred.clone(), P_pred.clone(), Inn_implicit.clone(), Pn.clone(), Q.clone(), R.clone(), dF_dX.clone(), dG_dX.clone(), dF_dU.clone(), X_kplus1, P_kplus1);

		return;
	}

	//Case when Implicit Measurements are empty
	if(Inn_implicit.rows == 0 && Z_explicit.rows>0){

		Explicit_Update(X_pred.clone(), P_pred.clone(), Z_explicit.clone(), Z_explicit_pred.clone(), Pn.clone(), Q.clone(), R_explicit.clone(), dF_dX.clone(), dH_dX.clone(), dF_dU.clone(), X_kplus1, P_kplus1);

		return;
	}


	//Case when Implicit and Explicit Measurements are empty
	if(Inn_implicit.rows == 0 && Z_explicit.rows == 0){

		X_kplus1 = X_pred.clone();
		P_kplus1 = P_pred.clone();
		return;

	}


	//At this point we know that we have both Implicit and Explicit Measurements
	  Mat Inn = Mat::zeros(Z_explicit.rows+Inn_implicit.rows,1,CV_64F);
	  Mat dInn_dX = Mat::zeros(dH_dX.rows+dG_dX.rows,dH_dX.cols,CV_64F);
	  Mat R = Mat::eye(Inn.rows,Inn.rows,CV_64F);

	  //Filling the Inn matrix with implicit and explicit measurements
	  Mat Inn_explicit = Z_explicit_pred.clone() - Z_explicit.clone();
	  for (int i=0; i<Inn_explicit.rows; i++){

		  Inn.at<double>(i,0) = Inn_explicit.clone().at<double>(i,0);
	  }

	  for (int i=0; i<Inn_implicit.rows; i++){

		  Inn.at<double>(i+Inn_explicit.rows,0) = Inn_implicit.clone().at<double>(i,0);
	  }

	  //Filling the dInn_dX matrix with implicit measurements covariance dG_dX and explicit measurements covariance dH_dX
	  for (int i=0; i<dH_dX.rows; i++){

		   for (int j=0; j<dH_dX.cols; j++){

		 	   dInn_dX.at<double>(i,j) = dH_dX.clone().at<double>(i,j);
		   }

	   }

	   for (int i=0; i<dG_dX.rows; i++){

		   for (int j=0; j<dG_dX.cols; j++){

			   dInn_dX.at<double>(i+dH_dX.rows,j) = dG_dX.clone().at<double>(i,j);
		   }

	   }


	   //Filling the measurement noise matrix R
	   for (int i=0; i<Inn_explicit.rows; i++){
		   R.at<double>(i,i) = R_explicit.clone().at<double>(i,i);
	   }

	   for (int i=0; i<Inn_implicit.rows; i++){
		   R.at<double>(i+Inn_explicit.rows,i+Inn_explicit.rows) = R_implicit.clone().at<double>(i,i);
	   }



	  Innovation_Update(X_pred.clone(), P_pred.clone(), Inn.clone(), Pn.clone(), Q.clone(), R.clone(), dF_dX.clone(), dInn_dX.clone(), dF_dU.clone(), X_kplus1, P_kplus1);
}

void Implicit_Explicit_Update(Mat X_pred, Mat P_pred, Mat Z_explicit, Mat Z_explicit_pred, Mat Inn_implicit, Mat Pn, Mat Q, Mat R_explicit, Mat R_implicit, Mat dF_dX,
							  Mat dH_dX, Mat dG_dX, Mat dF_dU, Mat &X_kplus1, Mat &P_kplus1, std::vector<double> &norm_inn_sq, bool &filter_converged, int norm_inn_sq_win, double convergence_threshold){

	//Case when Explicit Measurements are empty
	if(Inn_implicit.rows>0 && Z_explicit.rows == 0){

		Mat R = R_implicit.clone();

		//Innovation_Update(X_pred.clone(), P_pred.clone(), Inn_implicit.clone(), Pn.clone(), Q.clone(), R.clone(), dF_dX.clone(), dG_dX.clone(), dF_dU.clone(), X_kplus1, P_kplus1);
		Innovation_Update(X_pred.clone(), P_pred.clone(), Inn_implicit.clone(), Pn.clone(), Q.clone(), R.clone(), dF_dX.clone(), dG_dX.clone(), dF_dU.clone(), X_kplus1, P_kplus1,
                    norm_inn_sq, filter_converged, norm_inn_sq_win, convergence_threshold);

		return;
	}

	//Case when Implicit Measurements are empty
	if(Inn_implicit.rows == 0 && Z_explicit.rows>0){

		Explicit_Update(X_pred.clone(), P_pred.clone(), Z_explicit.clone(), Z_explicit_pred.clone(), Pn.clone(), Q.clone(), R_explicit.clone(), dF_dX.clone(), dH_dX.clone(), dF_dU.clone(), X_kplus1, P_kplus1);

		return;
	}


	//Case when Implicit and Explicit Measurements are empty
	if(Inn_implicit.rows == 0 && Z_explicit.rows == 0){

		X_kplus1 = X_pred.clone();
		P_kplus1 = P_pred.clone();
		return;

	}


	//At this point we know that we have both Implicit and Explicit Measurements
	  Mat Inn = Mat::zeros(Z_explicit.rows+Inn_implicit.rows,1,CV_64F);
	  Mat dInn_dX = Mat::zeros(dH_dX.rows+dG_dX.rows,dH_dX.cols,CV_64F);
	  Mat R = Mat::eye(Inn.rows,Inn.rows,CV_64F);

	  //Filling the Inn matrix with implicit and explicit measurements
	  Mat Inn_explicit = Z_explicit_pred.clone() - Z_explicit.clone();
	  for (int i=0; i<Inn_explicit.rows; i++){

		  Inn.at<double>(i,0) = Inn_explicit.clone().at<double>(i,0);
	  }

	  for (int i=0; i<Inn_implicit.rows; i++){

		  Inn.at<double>(i+Inn_explicit.rows,0) = Inn_implicit.clone().at<double>(i,0);
	  }

	  //Filling the dInn_dX matrix with implicit measurements covariance dG_dX and explicit measurements covariance dH_dX
	  for (int i=0; i<dH_dX.rows; i++){

		   for (int j=0; j<dH_dX.cols; j++){

		 	   dInn_dX.at<double>(i,j) = dH_dX.clone().at<double>(i,j);
		   }

	   }

	   for (int i=0; i<dG_dX.rows; i++){

		   for (int j=0; j<dG_dX.cols; j++){

			   dInn_dX.at<double>(i+dH_dX.rows,j) = dG_dX.clone().at<double>(i,j);
		   }

	   }


	   //Filling the measurement noise matrix R
	   for (int i=0; i<Inn_explicit.rows; i++){
		   R.at<double>(i,i) = R_explicit.clone().at<double>(i,i);
	   }

	   for (int i=0; i<Inn_implicit.rows; i++){
		   R.at<double>(i+Inn_explicit.rows,i+Inn_explicit.rows) = R_implicit.clone().at<double>(i,i);
	   }



	  Innovation_Update(X_pred.clone(), P_pred.clone(), Inn.clone(), Pn.clone(), Q.clone(), R.clone(), dF_dX.clone(), dInn_dX.clone(), dF_dU.clone(), X_kplus1, P_kplus1, norm_inn_sq, filter_converged, norm_inn_sq_win, convergence_threshold);
}




void Innovation_Update(Mat X_pred, Mat P_pred, Mat Inn, Mat Pn, Mat Q, Mat R, Mat dF_dX, Mat dInn_dX, Mat dF_dU,
			 Mat &X_kplus1, Mat &P_kplus1){

	//Matriz covariancia do processo de inovacao
	Mat S = (dInn_dX.clone() * P_pred.clone() * dInn_dX.clone().t()) + R.clone();
	if(isnan(S.at<double>(0,0)))
        {std::cout << "S has NaN values" << std::endl;exit(0);}

	//Ganho de Kalman
	Mat K = P_pred.clone() * dInn_dX.clone().t() * S.clone().inv();
	if(isnan(K.at<double>(0,0)))
	{std::cout << "K has NaN values" << std::endl;exit(0);}

	//Update

		//Update estado sistema
		//std::cout << (K.clone() * Inn.clone()) << std::endl << std::endl;
		X_kplus1 = X_pred.clone() - (K.clone() * Inn.clone());

		//Update covariancia sistema
		Mat Id = Mat::eye(P_pred.rows,P_pred.cols,CV_64F);
		Mat T = (Id.clone()-K.clone()*dInn_dX.clone());
		if(isnan(T.at<double>(0,0)))
            {std::cout << "T has NaN values" << std::endl;exit(0);}

		P_kplus1 = T.clone()*P_pred.clone()*T.clone().t();
		P_kplus1 = P_kplus1.clone() + K.clone()*R.clone()*K.clone() .t();
}

void Innovation_Update(Mat X_pred, Mat P_pred, Mat Inn, Mat Pn, Mat Q, Mat R, Mat dF_dX, Mat dInn_dX, Mat dF_dU,
			 Mat &X_kplus1, Mat &P_kplus1, std::vector<double> &mean_inn_vec, bool &filter_converged, int mean_inn_samples, double convergence_threshold){

	//Matriz covariancia do processo de inovacao
	Mat S = (dInn_dX.clone() * P_pred.clone() * dInn_dX.clone().t()) + R.clone();
	if(isnan(S.at<double>(0,0)))
        {std::cout << "S has NaN values" << std::endl;exit(0);}

	//Ganho de Kalman
	Mat K = P_pred.clone() * dInn_dX.clone().t() * S.clone().inv();
	if(isnan(K.at<double>(0,0)))
	{std::cout << "K has NaN values" << std::endl;exit(0);}

    //convergence analysis using the innovation vector
    double mean_inn = sum(Inn)[0]/Inn.rows;
	mean_inn = sqrt(mean_inn/2);
	if(mean_inn_vec.size() < mean_inn_samples)
	{
	    mean_inn_vec.push_back(mean_inn);
    }
    else
    {
        mean_inn_vec.erase (mean_inn_vec.begin());
        mean_inn_vec.push_back(mean_inn);
        Mat all_mean_inn = Mat(mean_inn_vec);
        double mean_mean_inn = mean(all_mean_inn)[0];
        //cout << mean_mean_inn << endl;
        if(mean_mean_inn < convergence_threshold)
        {
            filter_converged = true;
        }
        else
            filter_converged = false;
    }


	/*Mat norm_inn_sq_mat = Inn.t()*S.inv()*Inn;

	//cout << "mean_inn: " << mean_inn << endl;
	double norm_inn_sq_val = norm_inn_sq_mat.at<double>(0,0);
	if(norm_inn_sq.size() < norm_inn_sq_win)
	{
	    norm_inn_sq.push_back(norm_inn_sq_val);
    }
    else
    {
        norm_inn_sq.erase (norm_inn_sq.begin());
        norm_inn_sq.push_back(norm_inn_sq_val);
        Mat all_norm_inn_sq = Mat(norm_inn_sq);
        double norm_inn_sq_mean = mean(all_norm_inn_sq)[0];
        if(norm_inn_sq_mean < convergence_threshold)
        {
            filter_converged = true;
            cout << "converged: 2" << endl;
        }
        else
            filter_converged = false;
    }//*/

	//Update

		//Update estado sistema
		//std::cout << (K.clone() * Inn.clone()) << std::endl << std::endl;
		X_kplus1 = X_pred.clone() - (K.clone() * Inn.clone());

		//Update covariancia sistema
		Mat Id = Mat::eye(P_pred.rows,P_pred.cols,CV_64F);
		Mat T = (Id.clone()-K.clone()*dInn_dX.clone());
		if(isnan(T.at<double>(0,0)))
            {std::cout << "T has NaN values" << std::endl;exit(0);}

		P_kplus1 = T.clone()*P_pred.clone()*T.clone().t();
		P_kplus1 = P_kplus1.clone() + K.clone()*R.clone()*K.clone() .t();
}
