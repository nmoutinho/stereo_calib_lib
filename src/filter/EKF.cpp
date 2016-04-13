#include "filter/EKF.h"
#include "filter/Jacobians.h"
#include <iostream>

using namespace cv;


//FILTER PREDICTION
void EKF::Filter_Prediction(){

	//Prediction
	Prediction(X_k, U_k, P_k, Pn, Q, Rn_explicit, *this, X_pred, P_pred, Z_explicit_pred, Cache_dF_dX, Cache_dH_dX, Cache_dF_dU, S);

}

//FILTER UPDATE
void EKF::Filter_Update(Mat Z_explicit, Mat Z_implicit){

	Mat Inn_implicit = G(X_pred.clone(),Z_implicit.clone());

	Mat R_explicit = Rn_explicit.clone();
	Mat R_implicit;

	if(Inn_implicit.rows != 0){

		Cache_dG_dX = dG_dX(X_pred.clone(),Z_implicit.clone());

		Cache_dG_dZ = dG_dZ(X_pred.clone(), Z_implicit.clone());

		R_implicit = Cache_dG_dZ.clone()*Rn_implicit.clone()*Cache_dG_dZ.clone().t();

	}

	//Update
    Implicit_Explicit_Update(X_pred.clone(), P_pred.clone(), Z_explicit.clone(), Z_explicit_pred.clone(), Inn_implicit.clone(), Pn.clone(), Q.clone(), R_explicit.clone(), R_implicit.clone(), Cache_dF_dX.clone(), Cache_dH_dX.clone(), Cache_dG_dX.clone(), Cache_dF_dU.clone(),
		X_k, P_k, mean_inn_vec, filter_converged, mean_inn_samples, convergence_threshold);


}
