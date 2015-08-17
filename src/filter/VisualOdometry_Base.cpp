#include "filter/VisualOdometry_Base.h"
#include "filter/Jacobians.h"
#include "filter/EKF.h"
#include <iostream>

#pragma warning (disable : 4244) //disable conversion double to float

using namespace cv;

//Obtain the projection matrix P
cv::Mat ProjectionMat(cv::Mat CamCalibrationMat, cv::Mat TransformationMat){

	Mat T(3,4,CV_64F);
	for(int r=0; r<TransformationMat.rows-1; r++){
		for(int c=0; c<TransformationMat.cols; c++){

			T.at<double>(r,c) = TransformationMat.at<double>(r,c);
		}
	}

	Mat P = CamCalibrationMat*T;
	return P;
}

///Image Features to World Feature
cv::Point3d ImageToWorld(cv::Point2d LeftFeatureCoordinates, cv::Point2d RightFeatureCoordinates, cv::Mat Encoders, cv::Mat LeftCamCalibMat, cv::Mat RightCamCalibMat){

	KinTransforms KinTr;

	//Kinematics
	Kinematics_stereoCameras Kin;
	Kin.Apply(Encoders,KinTr);

	return ImageToWorld(LeftFeatureCoordinates, RightFeatureCoordinates, KinTr.UnifiedRefFrame_To_LeftCamRefFrame,
	KinTr.UnifiedRefFrame_To_RightCamRefFrame, LeftCamCalibMat, RightCamCalibMat);
}

///Image Features to World Feature
cv::Point3d ImageToWorld(cv::Point2d LeftFeatureCoordinates, cv::Point2d RightFeatureCoordinates,
cv::Mat UnifiedToLeft, cv::Mat UnifiedToRight, cv::Mat LeftCamCalibMat, cv::Mat RightCamCalibMat)
{

    cv::Point3d WorldPoint;
	Mat Pleft = ProjectionMat(LeftCamCalibMat, UnifiedToLeft);
	Mat Pright = ProjectionMat(RightCamCalibMat, UnifiedToRight);

	double ul = LeftFeatureCoordinates.x;
	double vl = LeftFeatureCoordinates.y;
	double ur = RightFeatureCoordinates.x;
	double vr = RightFeatureCoordinates.y;

	//Create matrix A - pag. 312 Multiple View Geometry in computer vision
	Mat A = Mat::zeros(4,4,CV_64F);

	Mat Row_0 = ul*Pleft.row(2) - Pleft.row(0);
	Mat Row_1 = vl*Pleft.row(2) - Pleft.row(1);
	Mat Row_2 = ur*Pright.row(2) - Pright.row(0);
	Mat Row_3 = vr*Pright.row(2) - Pright.row(1);

	A.at<double>(0,0)=Row_0.at<double>(0,0);
	A.at<double>(0,1)=Row_0.at<double>(0,1);
	A.at<double>(0,2)=Row_0.at<double>(0,2);
	A.at<double>(0,3)=Row_0.at<double>(0,3);

	A.at<double>(1,0)=Row_1.at<double>(0,0);
	A.at<double>(1,1)=Row_1.at<double>(0,1);
	A.at<double>(1,2)=Row_1.at<double>(0,2);
	A.at<double>(1,3)=Row_1.at<double>(0,3);

	A.at<double>(2,0)=Row_2.at<double>(0,0);
	A.at<double>(2,1)=Row_2.at<double>(0,1);
	A.at<double>(2,2)=Row_2.at<double>(0,2);
	A.at<double>(2,3)=Row_2.at<double>(0,3);

	A.at<double>(3,0)=Row_3.at<double>(0,0);
	A.at<double>(3,1)=Row_3.at<double>(0,1);
	A.at<double>(3,2)=Row_3.at<double>(0,2);
	A.at<double>(3,3)=Row_3.at<double>(0,3);

	//Apply an SVD to A and get the last column of V
	Mat V = SVD(A).vt.t().col(3);

	//Obtain the 3D coordinates of the point
	WorldPoint.x = V.at<double>(0,0)/V.at<double>(3,0);
	WorldPoint.y = V.at<double>(1,0)/V.at<double>(3,0);
	WorldPoint.z = V.at<double>(2,0)/V.at<double>(3,0);//*/

	return WorldPoint;
}

///World Feature to Image Features
void WorldToImage(const cv::Point3d &WorldPoint, cv::Mat Encoders, cv::Mat LeftCamCalibMat, cv::Mat RightCamCalibMat, cv::Point2d &LeftFeatureCoordinates,
				  cv::Point2d &RightFeatureCoordinates){

	KinTransforms KinTr;

	//Kinematics
	Kinematics_stereoCameras Kin;
	Kin.Apply(Encoders,KinTr);

	Mat WorldPointMat = Mat::ones(4,1,CV_64F);
	WorldPointMat.at<double>(0,0) = WorldPoint.x;
	WorldPointMat.at<double>(1,0) = WorldPoint.y;
	WorldPointMat.at<double>(2,0) = WorldPoint.z;

	Mat LeftPoint = KinTr.UnifiedRefFrame_To_LeftCamRefFrame*WorldPointMat;
	Mat RightPoint = KinTr.UnifiedRefFrame_To_RightCamRefFrame*WorldPointMat;

	Mat NormLeftPoint = Mat::ones(3,1,CV_64F);
	NormLeftPoint.at<double>(0,0) = LeftPoint.at<double>(0,0)/LeftPoint.at<double>(2,0);
	NormLeftPoint.at<double>(1,0) = LeftPoint.at<double>(1,0)/LeftPoint.at<double>(2,0);

	Mat NormRightPoint = Mat::ones(3,1,CV_64F);
	NormRightPoint.at<double>(0,0) = RightPoint.at<double>(0,0)/RightPoint.at<double>(2,0);
	NormRightPoint.at<double>(1,0) = RightPoint.at<double>(1,0)/RightPoint.at<double>(2,0);

	Mat ImgLeftPoint = LeftCamCalibMat*NormLeftPoint;
	Mat ImgRightPoint = RightCamCalibMat*NormRightPoint;

	LeftFeatureCoordinates.x = (ImgLeftPoint.at<double>(0,0));
	LeftFeatureCoordinates.y = (ImgLeftPoint.at<double>(1,0));

	RightFeatureCoordinates.x = (ImgRightPoint.at<double>(0,0));
	RightFeatureCoordinates.y = (ImgRightPoint.at<double>(1,0));
}

//Find the minimum distance between two lines
double MinimumLinesDistance(cv::Point2d LeftFeatureCoordinates, cv::Point2d RightFeatureCoordinates, cv::Mat Encoders, cv::Mat LeftCamCalibMat, cv::Mat RightCamCalibMat){

	//Kinematic transform
	KinTransforms KinTr;

	//Kinematics
	Kinematics_stereoCameras Kin;
	Kin.Apply(Encoders,KinTr);

	//Obtain the normalized coordinates of the left and right features
	Mat LeftFeature = Mat::ones(3,1,CV_64F);
	LeftFeature.at<double>(0,0) = LeftFeatureCoordinates.x;
	LeftFeature.at<double>(1,0) = LeftFeatureCoordinates.y;

	Mat RightFeature = Mat::ones(3,1,CV_64F);
	RightFeature.at<double>(0,0) = RightFeatureCoordinates.x;
	RightFeature.at<double>(1,0) = RightFeatureCoordinates.y;

	Mat LeftFeatureNorm = LeftCamCalibMat.inv()*LeftFeature;
	Mat RightFeatureNorm = RightCamCalibMat.inv()*RightFeature;

	//Obtain the coordinates of the points P0, P1 and Q0 and Q1
	Mat P0 = Mat::zeros(3,1,CV_64F);
	Mat P1 = LeftFeatureNorm.clone();
	Mat Q0 = Mat::zeros(3,1,CV_64F);
	Mat Q1 = Mat::zeros(3,1,CV_64F);

	Mat RightRefFrame_To_LeftRefFrame = KinTr.UnifiedRefFrame_To_RightCamRefFrame.inv()*KinTr.UnifiedRefFrame_To_LeftCamRefFrame;

	Mat RightPoint = Mat::ones(4,1,CV_64F);
	RightPoint.at<double>(0,0) = RightFeatureNorm.at<double>(0,0);
	RightPoint.at<double>(1,0) = RightFeatureNorm.at<double>(1,0);
	Mat RightFeature_On_LeftRefFrame = RightRefFrame_To_LeftRefFrame*RightPoint;

	Mat OriginPoint = Mat::zeros(4,1,CV_64F);
	OriginPoint.at<double>(3,0) = 1;
	Mat OriginPoint_On_LeftRefFrame = RightRefFrame_To_LeftRefFrame*OriginPoint;

	Q0.at<double>(0,0) = OriginPoint_On_LeftRefFrame.at<double>(0,0);
	Q0.at<double>(1,0) = OriginPoint_On_LeftRefFrame.at<double>(1,0);
	Q1.at<double>(0,0) = RightFeature_On_LeftRefFrame.at<double>(0,0);
	Q1.at<double>(1,0) = RightFeature_On_LeftRefFrame.at<double>(1,0);

	Mat u = P1-P0;
	Mat v = Q1-Q0;
	Mat w0 = P0-Q0;

	Mat a_mat = u.t()*u;
	Mat b_mat = u.t()*v;
	Mat c_mat = v.t()*v;
	Mat d_mat = u.t()*w0;
	Mat e_mat = v.t()*w0;

	double a = a_mat.at<double>(0,0);
	double b = b_mat.at<double>(0,0);
	double c = c_mat.at<double>(0,0);
	double d = d_mat.at<double>(0,0);
	double e = e_mat.at<double>(0,0);

	Mat MinDistance = (w0) + (b*e-c*d)*u/(a*c-b*b) - (a*e-b*d)*v/(a*c-b*b);
	double MinDistanceNorm = cv::norm(MinDistance);

	return MinDistanceNorm;

}

//Remove bad measurements from Zpred and dH_dX
void EliminateMeasurements(int FixedMeasurements, std::vector<int> FeaturesToEliminateIndx, Mat &Zpred, Mat &dH_dX){

	int NewDim = Zpred.rows - 4*FeaturesToEliminateIndx.size();
	int NumFeats = (Zpred.rows - FixedMeasurements)/4;

	Mat AdapatedZpred(NewDim,1,CV_64F);
	Mat AdapatedDH_dX(NewDim, dH_dX.cols,CV_64F);

	//Copy the values for the fixed measurements
	for (int i=0; i<FixedMeasurements; i++){
		AdapatedZpred.at<double>(i,0) = Zpred.at<double>(i,0);

		for(int j=0; j<dH_dX.cols; j++){
			AdapatedDH_dX.at<double>(i,j) = dH_dX.at<double>(i,j);
		}
	}

	int it=0;
	int pos=0;

	if(FeaturesToEliminateIndx.size()>0){

		for(int i=0; i<NumFeats; i++){

			if(i==FeaturesToEliminateIndx[it]){

				if(it<FeaturesToEliminateIndx.size()-1)
					it++;
			}

			else{
				AdapatedZpred.at<double>(FixedMeasurements + 4*pos,0)	= Zpred.at<double>(FixedMeasurements + 4*i,0);
				AdapatedZpred.at<double>(FixedMeasurements + 4*pos+1,0) = Zpred.at<double>(FixedMeasurements + 4*i+1,0);
				AdapatedZpred.at<double>(FixedMeasurements + 4*pos+2,0) = Zpred.at<double>(FixedMeasurements + 4*i+2,0);
				AdapatedZpred.at<double>(FixedMeasurements + 4*pos+3,0) = Zpred.at<double>(FixedMeasurements + 4*i+3,0);

				for(int j=0; j<dH_dX.cols; j++){
					AdapatedDH_dX.at<double>(FixedMeasurements + 4*pos,j)	= dH_dX.at<double>(FixedMeasurements + 4*i,j);
					AdapatedDH_dX.at<double>(FixedMeasurements + 4*pos+1,j) = dH_dX.at<double>(FixedMeasurements + 4*i+1,j);
					AdapatedDH_dX.at<double>(FixedMeasurements + 4*pos+2,j) = dH_dX.at<double>(FixedMeasurements + 4*i+2,j);
					AdapatedDH_dX.at<double>(FixedMeasurements + 4*pos+3,j) = dH_dX.at<double>(FixedMeasurements + 4*i+3,j);
				}
				pos++;
			}

		}

		Zpred.release();
		dH_dX.release();

		Zpred = AdapatedZpred.clone();
		dH_dX = AdapatedDH_dX.clone();
	}
}

//Adjust system state
void AdjustSystemState(int FixedStateParameters, std::vector<int> FeaturesToReplaceIndx, std::vector<cv::Point2f> LeftFeaturesNew, std::vector<cv::Point2f> RightFeaturesNew,
					   double FeatureNoise, cv::Mat &X, cv::Mat &P){

						   for (int i=0; i<FeaturesToReplaceIndx.size(); i++) {

							   X.at<double>(FixedStateParameters + 4*FeaturesToReplaceIndx[i],0) = LeftFeaturesNew[i].x;
							   X.at<double>(FixedStateParameters + 4*FeaturesToReplaceIndx[i] + 1,0) = LeftFeaturesNew[i].y;
							   X.at<double>(FixedStateParameters + 4*FeaturesToReplaceIndx[i] + 2,0) = RightFeaturesNew[i].x;
							   X.at<double>(FixedStateParameters + 4*FeaturesToReplaceIndx[i] + 3,0) = RightFeaturesNew[i].y;

							   for (int j=0; j<P.cols; j++) {

								   P.at<double>(FixedStateParameters + 4*FeaturesToReplaceIndx[i],j) = -1000;
								   P.at<double>(FixedStateParameters + 4*FeaturesToReplaceIndx[i] + 1,j) = -1000;
								   P.at<double>(FixedStateParameters + 4*FeaturesToReplaceIndx[i] + 2,j) = -1000;
								   P.at<double>(FixedStateParameters + 4*FeaturesToReplaceIndx[i] + 3,j) = -1000;
							   }

							   for (int j=0; j<P.rows; j++) {

								   P.at<double>(j, FixedStateParameters + 4*FeaturesToReplaceIndx[i]) = -1000;
								   P.at<double>(j, FixedStateParameters + 4*FeaturesToReplaceIndx[i] + 1) = -1000;
								   P.at<double>(j, FixedStateParameters + 4*FeaturesToReplaceIndx[i] + 2) = -1000;
								   P.at<double>(j, FixedStateParameters + 4*FeaturesToReplaceIndx[i] + 3) = -1000;
							   }

							   P.at<double>(FixedStateParameters + 4*FeaturesToReplaceIndx[i],FixedStateParameters + 4*FeaturesToReplaceIndx[i]) = FeatureNoise*FeatureNoise;
							   P.at<double>(FixedStateParameters + 4*FeaturesToReplaceIndx[i] + 1,FixedStateParameters + 4*FeaturesToReplaceIndx[i] + 1) = FeatureNoise*FeatureNoise;
							   P.at<double>(FixedStateParameters + 4*FeaturesToReplaceIndx[i] + 2,FixedStateParameters + 4*FeaturesToReplaceIndx[i] + 2) = FeatureNoise*FeatureNoise;
							   P.at<double>(FixedStateParameters + 4*FeaturesToReplaceIndx[i] + 3,FixedStateParameters + 4*FeaturesToReplaceIndx[i] + 3) = FeatureNoise*FeatureNoise;
						   }

}

//Obtain the Epipolar Line parameters to find the feature - "EpipolarLineParams = [A B C]^T"
cv::Mat EpipolarLine(const cv::Mat &FundamentalMat, const cv::Mat &ImageFeature){

	Mat EpipolarLineParams(1,3,CV_64F);

	double f11 = FundamentalMat.at<double>(0,0);
	double f12 = FundamentalMat.at<double>(0,1);
	double f13 = FundamentalMat.at<double>(0,2);
	double f21 = FundamentalMat.at<double>(1,0);
	double f22 = FundamentalMat.at<double>(1,1);
	double f23 = FundamentalMat.at<double>(1,2);
	double f31 = FundamentalMat.at<double>(2,0);
	double f32 = FundamentalMat.at<double>(2,1);
	double f33 = FundamentalMat.at<double>(2,2);

	double ul = ImageFeature.at<float>(0,0);
	double vl = ImageFeature.at<float>(1,0);

	//A
	double A = f31 + f11*ul + f21*vl;
	//B
	double B = f32 + f12*ul + f22*vl;
	//C
	double C = f33 + f13*ul + f23*vl;

	double NormVal = std::sqrt(double(A*A + B*B));

	EpipolarLineParams.at<double>(0,0) = A/NormVal;
	EpipolarLineParams.at<double>(0,1) = B/NormVal;
	EpipolarLineParams.at<double>(0,2) = C/NormVal;

	return EpipolarLineParams;
}//*/


//Obtain the Distance to the Epipolar line - "EpipolarLineParams = [A B C]^T"
double DistanceToEpipolar(const cv::Mat &K_Left, const cv::Mat &K_Right, const cv::Mat &Tr_Left_To_Right, const cv::Mat &Tr_Right_To_Left,
						  const cv::Mat &Point_Left, const cv::Mat &Point_Right){

	double Distance = 0;

	//For transformation Right to Left
	Mat R_Right_To_Left(3,3,CV_64F);
	Mat T_Right_To_Left = Mat::zeros(3,3,CV_64F);
	Mat t_Right_To_Left(3,1,CV_64F);

	for(int i=0; i<3; i++)
	{
		t_Right_To_Left.at<double>(i,0) = Tr_Right_To_Left.at<double>(i,3);

		for(int j=0; j<3; j++)
		{
			R_Right_To_Left.at<double>(i,j) = Tr_Right_To_Left.at<double>(i,j);
		}
	}

	T_Right_To_Left.at<double>(0,1) = -t_Right_To_Left.at<double>(2,0);
	T_Right_To_Left.at<double>(0,2) = t_Right_To_Left.at<double>(1,0);
	T_Right_To_Left.at<double>(1,0) = t_Right_To_Left.at<double>(2,0);
	T_Right_To_Left.at<double>(1,2) = -t_Right_To_Left.at<double>(0,0);
	T_Right_To_Left.at<double>(2,0) = -t_Right_To_Left.at<double>(1,0);
	T_Right_To_Left.at<double>(2,1) = t_Right_To_Left.at<double>(0,0);

	Mat F_Right_To_Left = (K_Left.inv()).t()*R_Right_To_Left*T_Right_To_Left*K_Right.inv();
	Mat D = EpipolarLine(F_Right_To_Left, Point_Left)*Point_Right;
	Distance = Distance + abs(D.at<double>(0,0));

	//For transformation Left to Right
	Mat R_Left_To_Right(3,3,CV_64F);
	Mat T_Left_To_Right = Mat::zeros(3,3,CV_64F);
	Mat t_Left_To_Right(3,1,CV_64F);

	for(int i=0; i<3; i++)
	{
		t_Left_To_Right.at<double>(i,0) = Tr_Left_To_Right.at<double>(i,3);

		for(int j=0; j<3; j++)
		{
			R_Left_To_Right.at<double>(i,j) = Tr_Left_To_Right.at<double>(i,j);
		}
	}

	T_Left_To_Right.at<double>(0,1) = -t_Left_To_Right.at<double>(2,0);
	T_Left_To_Right.at<double>(0,2) = t_Left_To_Right.at<double>(1,0);
	T_Left_To_Right.at<double>(1,0) = t_Left_To_Right.at<double>(2,0);
	T_Left_To_Right.at<double>(1,2) = -t_Left_To_Right.at<double>(0,0);
	T_Left_To_Right.at<double>(2,0) = -t_Left_To_Right.at<double>(1,0);
	T_Left_To_Right.at<double>(2,1) = t_Left_To_Right.at<double>(0,0);

	Mat F_Left_To_Right = (K_Right.inv()).t()*R_Left_To_Right*T_Left_To_Right*K_Left.inv();
	D = EpipolarLine(F_Left_To_Right, Point_Right)*Point_Left;
	Distance = Distance + abs(D.at<double>(0,0));

	return Distance;
}//*/

double DistanceToEpipolar(cv::Point2f Lpoint, cv::Point2f Rpoint, cv::Mat FundamentalMat)
{
    Mat LpointMat = Mat::ones(3,1,CV_64F);
    Mat RpointMat = Mat::ones(3,1,CV_64F);

    LpointMat.at<double>(0,0) = Lpoint.x;
    LpointMat.at<double>(1,0) = Lpoint.y;

    RpointMat.at<double>(0,0) = Rpoint.x;
    RpointMat.at<double>(1,0) = Rpoint.y;

    Mat Epipolar = FundamentalMat.clone()*LpointMat.clone();
    double A = Epipolar.clone().at<double>(0,0);
    double B = Epipolar.clone().at<double>(1,0);
    double C = Epipolar.clone().at<double>(2,0);

    double Anew = A/sqrt(A*A+B*B);
    double Bnew = B/sqrt(A*A+B*B);
    double Cnew = C/sqrt(A*A+B*B);

    Epipolar.at<double>(0,0) = Anew;
    Epipolar.at<double>(1,0) = Bnew;
    Epipolar.at<double>(2,0) = Cnew;

    Mat ErrorMat = RpointMat.clone().t()*Epipolar.clone();
    return ErrorMat.clone().at<double>(0,0);
}

//Returns a matrix of this form [R | t]
EssentialMatFactorizationData EssentialMatFactorization(std::vector<cv::Point2f> LeftPoints, std::vector<cv::Point2f> RightPoints, cv::Mat Kleft, cv::Mat Kright)
{
    EssentialMatFactorizationData Data;
    bool CheckF = false;

    Mat F = findFundamentalMat(Mat(RightPoints), Mat(LeftPoints), CV_FM_8POINT);
    Mat E = Kleft.t()*F*Kright;

    if(CheckF)
    {
        Mat ImL = Mat::zeros(240, 320, CV_8UC3);
        Mat ImR = Mat::zeros(240, 320, CV_8UC3);

        for(int k=0; k<LeftPoints.size(); k++)
        {
            circle(ImL, LeftPoints[k], 2, Scalar(255,0,255), -1);
            circle(ImR, RightPoints[k], 2, Scalar(255,0,255), -1);
            std::string s;
            std::stringstream out;
            out << k;
            s = out.str();
            putText(ImL, s, LeftPoints[k], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
            putText(ImR, s, RightPoints[k], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        }

        std::vector<cv::Vec3f> lines_l;
        computeCorrespondEpilines(Mat(LeftPoints), 1, F, lines_l);
        for(vector<cv::Vec3f>::const_iterator it=lines_l.begin(); it!=lines_l.end(); ++it)
        {
            cv::line(ImR, Point(0, -(*it)[2]/(*it)[1]), Point(ImR.cols, -((*it)[2]+(*it)[0]*ImR.cols)/(*it)[1]), Scalar(255,255,255));
        }

        imshow("ImL",ImL);
        imshow("ImR",ImR);
        waitKey(1);
    }

    SVD svd;
    Mat u, s, vt;
    svd.compute(E, s, u, vt);

    //For Tx
    Mat final_tx, final_rotations;

    Mat Tx(3,1,CV_64F);
    Tx.at<double>(0,0) = u.at<double>(0,2);
    Tx.at<double>(1,0) = u.at<double>(1,2);
    Tx.at<double>(2,0) = u.at<double>(2,2);

    Mat sum = Mat::zeros(1,1,CV_64F);
    for(int k=0; k<LeftPoints.size(); k++)
    {
        Mat PtR = Mat::ones(3,1,CV_64F);
        Mat PtL = Mat::ones(3,1,CV_64F);

        PtL.at<double>(0,0) = LeftPoints[k].x;
        PtL.at<double>(1,0) = LeftPoints[k].y;

        PtR.at<double>(0,0) = RightPoints[k].x;
        PtR.at<double>(1,0) = RightPoints[k].y;

        Mat NormPtR = Kright.inv()*PtR;
        Mat NormPtL = Kleft.inv()*PtL;

        Mat Cross = Tx.cross(NormPtR);
        Mat Aux = E*NormPtL;

        sum = sum + Cross.t()*Aux;
    }

    if(sum.at<double>(0,0) < 0)
        Data.t = -Tx.clone();
    else
        Data.t = Tx.clone();


    Mat E1(3,1,CV_64F);
    Mat E2(3,1,CV_64F);
    Mat E3(3,1,CV_64F);
    for(int k=0; k<3; k++)
    {
        E1.at<double>(k,0) = E.clone().at<double>(k,0);
        E2.at<double>(k,0) = E.clone().at<double>(k,1);
        E3.at<double>(k,0) = E.clone().at<double>(k,2);
    }

    Mat E1_Tx = E1.clone().cross(Data.t.clone());
    Mat E2_E3 = E2.clone().cross(E3.clone());
    Mat E2_Tx = E2.clone().cross(Data.t.clone());
    Mat E3_E1 = E3.clone().cross(E1.clone());
    Mat E3_Tx = E3.clone().cross(Data.t.clone());
    Mat E1_E2 = E1.clone().cross(E2.clone());

    Mat W1 = E1_Tx.clone()+E2_E3.clone();
    Mat W2 = E2_Tx.clone()+E3_E1.clone();
    Mat W3 = E3_Tx.clone()+E1_E2.clone();

    Mat R(3,3,CV_64F);
    for(int k=0; k<3; k++)
    {
        R.at<double>(k,0) = W1.clone().at<double>(k,0);
        R.at<double>(k,1) = W2.clone().at<double>(k,0);
        R.at<double>(k,2) = W3.clone().at<double>(k,0);
    }

    svd.compute(R, s, u, vt);
    Data.R = u*vt;

    /*Mat Transformation = Mat::eye(4,4,CV_64F);
    for(int r=0; r<3; r++)
    {
        Transformation.at<double>(r,3) = Data.t.at<double>(r,0);

        for(int c=0; c<3; c++)
        {
            Transformation.at<double>(r,c) = Data.R.at<double>(r,c);
        }
    }

    for(int k=0; k<LeftPoints.size(); k++)
    {
        Mat PtR = Mat::ones(3,1,CV_64F);
        Mat PtL = Mat::ones(3,1,CV_64F);

        PtL.at<double>(0,0) = LeftPoints[k].x;
        PtL.at<double>(1,0) = LeftPoints[k].y;

        PtR.at<double>(0,0) = RightPoints[k].x;
        PtR.at<double>(1,0) = RightPoints[k].y;

        Mat NormPtR_Mat = Kright.inv()*PtR;
        Mat NormPtL_Mat = Kleft.inv()*PtL;

        Point2f NormPtL, NormPtR;
        NormPtL.x = NormPtL_Mat.at<double>(0,0);
        NormPtL.y = NormPtL_Mat.at<double>(1,0);

        NormPtR.x = NormPtR_Mat.at<double>(0,0);
        NormPtR.y = NormPtR_Mat.at<double>(1,0);

        Mat LeftWorldPt, RightWorldPt;
        ImagePointToWorldPoint(NormPtL, NormPtR, Transformation, LeftWorldPt, RightWorldPt);
    }//*/

    return Data;

}//*/

//Returns a matrix of this form [R | t]
/*EssentialMatFactorizationData EssentialMatFactorization(std::vector<cv::Point2f> LeftPoints, std::vector<cv::Point2f> RightPoints, cv::Mat Kleft, cv::Mat Kright)
{
    EssentialMatFactorizationData Data;

    Mat LeftPointsMat = Mat(LeftPoints);
    Mat RightPointsMat = Mat(RightPoints);

    std::cout << LeftPointsMat << std::endl;
    std::cout << RightPointsMat << std::endl;

    Mat F = findFundamentalMat(LeftPointsMat, RightPointsMat, CV_FM_8POINT);
    Mat E = Kright.t()*F*Kleft;

    SVD svd;
    Mat u, s, vt;
    svd.compute(E, s, u, vt);

    Mat W = Mat::zeros(3,3,CV_64F);
    W.at<double>(0,1) = -1;
    W.at<double>(1,0) = 1;
    W.at<double>(2,2) = 1;

    Mat Z = Mat::zeros(3,3,CV_64F);
    Z.at<double>(0,1) = 1;
    Z.at<double>(1,0) = -1;

    Mat T1 = Mat::eye(4,4,CV_64F);
    Mat S1 = u*Z*u.t();
    Mat t1(3,1,CV_64F);
    t1.at<double>(0,0) = -S1.at<double>(1,2);
    t1.at<double>(1,0) = S1.at<double>(0,2);
    t1.at<double>(2,0) = -S1.at<double>(0,1);
    Mat R1 = u*W*vt;

    Mat T2 = Mat::eye(4,4,CV_64F);
    Mat S2 = -u*Z*u.t();
    Mat t2(3,1,CV_64F);
    t2.at<double>(0,0) = -S2.at<double>(1,2);
    t2.at<double>(1,0) = S2.at<double>(0,2);
    t2.at<double>(2,0) = -S2.at<double>(0,1);
    Mat R2 = u*W*vt;

    Mat T3 = Mat::eye(4,4,CV_64F);
    Mat S3 = u*Z*u.t();
    Mat t3(3,1,CV_64F);
    t3.at<double>(0,0) = -S3.at<double>(1,2);
    t3.at<double>(1,0) = S3.at<double>(0,2);
    t3.at<double>(2,0) = -S3.at<double>(0,1);
    Mat R3 = u*W.t()*vt;

    Mat T4 = Mat::eye(4,4,CV_64F);
    Mat S4 = -u*Z*u.t();
    Mat t4(3,1,CV_64F);
    t4.at<double>(0,0) = -S4.at<double>(1,2);
    t4.at<double>(1,0) = S4.at<double>(0,2);
    t4.at<double>(2,0) = -S4.at<double>(0,1);
    Mat R4 = u*W.t()*vt;

    for(int r=0; r<3; r++)
    {
        T1.at<double>(r,3) = t1.at<double>(r,0);
        T2.at<double>(r,3) = t2.at<double>(r,0);
        T3.at<double>(r,3) = t3.at<double>(r,0);
        T4.at<double>(r,3) = t4.at<double>(r,0);

        for(int c=0; c<3; c++)
        {
            T1.at<double>(r,c) = R1.at<double>(r,c);
            T2.at<double>(r,c) = R2.at<double>(r,c);
            T3.at<double>(r,c) = R3.at<double>(r,c);
            T4.at<double>(r,c) = R4.at<double>(r,c);

        }
    }

    Mat rot;
    Rodrigues(R1, rot);
    std::cout << "1: " << rot*180./CV_PI << " " << t1 << std::endl;
    Rodrigues(R2, rot);
    std::cout << "2: " <<rot*180./CV_PI << " " << t2 << std::endl;
    Rodrigues(R3, rot);
    std::cout << "3: " <<rot*180./CV_PI << " " << t3 << std::endl;
    Rodrigues(R4, rot);
    std::cout << "4: " <<rot*180./CV_PI << " " << t4 << std::endl;

    Mat Pt1_l = ImagePointToWorldPoint(LeftPoints[1], RightPoints[1], T1, Kleft, Kright);
    Mat Pt2_l = ImagePointToWorldPoint(LeftPoints[1], RightPoints[1], T2, Kleft, Kright);
    Mat Pt3_l = ImagePointToWorldPoint(LeftPoints[1], RightPoints[1], T3, Kleft, Kright);
    Mat Pt4_l = ImagePointToWorldPoint(LeftPoints[1], RightPoints[1], T4, Kleft, Kright);

    Mat Pt1_r = ImagePointToWorldPoint(RightPoints[1], LeftPoints[1], T1.inv(), Kright, Kleft);
    Mat Pt2_r = ImagePointToWorldPoint(RightPoints[1], LeftPoints[1], T2.inv(), Kright, Kleft);
    Mat Pt3_r = ImagePointToWorldPoint(RightPoints[1], LeftPoints[1], T3.inv(), Kright, Kleft);
    Mat Pt4_r = ImagePointToWorldPoint(RightPoints[1], LeftPoints[1], T4.inv(), Kright, Kleft);

    if(Pt1_l.at<double>(2,0) > 0 && Pt1_r.at<double>(2,0) > 0)
    {
        Data.R = R1;
        Data.t = t1;
    }
    else if(Pt2_l.at<double>(2,0) > 0 && Pt2_r.at<double>(2,0) > 0)
    {
        Data.R = R2;
        Data.t = t2;
    }
    else if(Pt3_l.at<double>(2,0) > 0 && Pt3_r.at<double>(2,0) > 0)
    {
        Data.R = R3;
        Data.t = t3;
    }
    else if(Pt4_l.at<double>(2,0) > 0 && Pt4_r.at<double>(2,0) > 0)
    {
        Data.R = R4;
        Data.t = t4;
    }

    return Data;

}//*/


Mat ImagePointToWorldPoint(cv::Point2f ImagePointL, cv::Point2f ImagePointR, Mat Transformation_LtoR, Mat CalibMatL, Mat CalibMatR)
{

	Mat Pleft = ProjectionMat(CalibMatL, Mat::eye(4,4,CV_64F));
	Mat Pright = ProjectionMat(CalibMatR, Transformation_LtoR);

	double ul = ImagePointL.x;
	double vl = ImagePointL.y;
	double ur = ImagePointR.x;
	double vr = ImagePointR.y;

	//Create matrix A - pag. 312 Multiple View Geometry in computer vision
	Mat A = Mat::zeros(4,4,CV_64F);

	Mat Row_0 = ul*Pleft.row(2) - Pleft.row(0);
	Mat Row_1 = vl*Pleft.row(2) - Pleft.row(1);
	Mat Row_2 = ur*Pright.row(2) - Pright.row(0);
	Mat Row_3 = vr*Pright.row(2) - Pright.row(1);

	A.at<double>(0,0)=Row_0.at<double>(0,0);
	A.at<double>(0,1)=Row_0.at<double>(0,1);
	A.at<double>(0,2)=Row_0.at<double>(0,2);
	A.at<double>(0,3)=Row_0.at<double>(0,3);

	A.at<double>(1,0)=Row_1.at<double>(0,0);
	A.at<double>(1,1)=Row_1.at<double>(0,1);
	A.at<double>(1,2)=Row_1.at<double>(0,2);
	A.at<double>(1,3)=Row_1.at<double>(0,3);

	A.at<double>(2,0)=Row_2.at<double>(0,0);
	A.at<double>(2,1)=Row_2.at<double>(0,1);
	A.at<double>(2,2)=Row_2.at<double>(0,2);
	A.at<double>(2,3)=Row_2.at<double>(0,3);

	A.at<double>(3,0)=Row_3.at<double>(0,0);
	A.at<double>(3,1)=Row_3.at<double>(0,1);
	A.at<double>(3,2)=Row_3.at<double>(0,2);
	A.at<double>(3,3)=Row_3.at<double>(0,3);

	//Apply an SVD to A and get the last column of V
	Mat V = SVD(A).vt.t().col(3);

	//Obtain the 3D coordinates of the point
	Mat WorldPoint(3,1,CV_64F);
	WorldPoint.at<double>(0,0) = V.at<double>(0,0)/V.at<double>(3,0);
	WorldPoint.at<double>(1,0) = V.at<double>(1,0)/V.at<double>(3,0);
	WorldPoint.at<double>(2,0) = V.at<double>(2,0)/V.at<double>(3,0);//*/

	return WorldPoint;

}

void OpticalFlowAnalysis(std::vector<cv::Point2f> LeftPoints, std::vector<cv::Point2f> RightPoints, cv::Mat Kleft, cv::Mat Kright, int ImgW, int ImgH)
{
    Mat OpFlow = Mat::zeros(ImgH, ImgW, CV_8UC1);

    for(int i=0; i<LeftPoints.size(); i++)
    {
        line(OpFlow, LeftPoints[i], RightPoints[i], 255);
    }

    imshow("OpFlow",OpFlow);
    waitKey(1);

}
