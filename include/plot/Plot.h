#ifndef _PLOT_H_
#define _PLOT_H_

#include <opencv/cv.h>
#include <vector>

class Plot{

	public:

		Plot(cv::Mat Data, const char * FigureName, int FigureWidth=600, int FigureHeight=400, 
			double XAxisMin=1e8, double XAxisMax=1e8, double YAxisMin=1e8, double YAxisMax=1e8);
		Plot(std::vector<double> Data, const char * FigureName, int FigureWidth=600, int FigureHeight=400, 
			double XAxisMin=1e8, double XAxisMax=1e8, double YAxisMin=1e8, double YAxisMax=1e8);
		Plot(cv::Mat Xdata, cv::Mat Ydata, const char * FigureName, int FigureWidth=600, int FigureHeight=400, 
			double XAxisMin=1e8, double XAxisMax=1e8, double YAxisMin=1e8, double YAxisMax=1e8);
		Plot(std::vector<double> Xdata, std::vector<double> Ydata, const char * FigureName, int FigureWidth=600, int FigureHeight=400, 
			double XAxisMin=1e8, double XAxisMax=1e8, double YAxisMin=1e8, double YAxisMax=1e8);
		
		void ConstructorHelper(cv::Mat Xdata, cv::Mat Ydata, const char * FigureName, int FigureWidth, int FigureHeight, double XAxisMin, double XAxisMax, double YAxisMin, double YAxisMax);
		void ConstructorHelper(cv::Mat Data, const char * FigureName, int FigureWidth, int FigureHeight, double XAxisMin, double XAxisMax, double YAxisMin, double YAxisMax);
};


//########################################### Functions ########################################### 
cv::Mat AuxFunc_LinearInterpolation(double Xa, double Xb, double Ya, double Yb, cv::Mat Xdata);
void    AuxFunc_DrawAxis(double MinX, double MaxX, double MinY, double MaxY, double ImageXzero, double ImageYzero, double CurrentX, double CurrentY, int NumVecElements, int FigW, int FigH, cv::Mat &FigureRed, cv::Mat &FigureGreen, cv::Mat &FigureBlue);
void    AuxFunc_DrawValuesAsText(double Value, int Xloc, int Yloc, int XMargin, int YMargin, cv::Mat &FigureRed, cv::Mat &FigureGreen, cv::Mat &FigureBlue);
void    AuxFunc_DrawValuesAsText(const char * Text, double Value, int Xloc, int Yloc, int XMargin, int YMargin, cv::Mat &FigureRed, cv::Mat &FigureGreen, cv::Mat &FigureBlue);
void    AuxFunc_DrawLine(int Xstart, int Xend, int Ystart, int Yend, cv::Mat &FigureRed, cv::Mat &FigureGreen, cv::Mat &FigureBlue, int LineWidth, cv::Point3d RGBLineColor);

#endif
