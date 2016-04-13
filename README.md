# stereo_calib_lib
Stereo calibration library for C++ (implemented by Nuno Moutinho)

The stereo_calib_lib calibrates any stereo vision system in an online manner. The system assumes the baseline is fixed between the two cameras and estimates the 6 parameters that compose the transformation between the left and right cameras: Tx, Ty, Tz, Rx, Ry and Rz. 

This system receives image pairs (from left and right cameras) and can be left running during operation, providing a calibrated transformation between the two cameras at any time instance, in an online manner. The system adapts to sudden changes in the cameras' configuration and can be used with active vision (version and vergence).

## Requirements

To use stereo_calib_lib you need the follwing libraries:

 - gcc
 - Cmake >= 2.8
 - OpenCv >= 2.4.9
 - Boost

## Configure the system (on Ubuntu 12.04 LTS or 14.04 LTS) - installing gcc, Boost, Cmake and OpenCv 2.4.9

OK, so the first step is to make sure that everything in the system is updated and upgraded. Open the terminal and write this:

	>> sudo apt-get update
	>> sudo apt-get upgrade

Now, you need to install many dependencies, such as support for reading and writing image files, drawing on the screen, some needed tools, other libraries, etc... This step is very easy, you only need to write the following command in the Terminal:

	>> sudo apt-get install build-essential libgtk2.0-dev libjpeg-dev libtiff4-dev libjasper-dev libopenexr-dev cmake python-dev python-numpy python-tk libtbb-dev libeigen3-dev yasm libfaac-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev libqt4-dev libqt4-opengl-dev sphinx-common texlive-latex-extra libv4l-dev libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev default-jdk ant libvtk5-qt4-dev

Install boost in your system:

	>> sudo apt-get install libboost-all-dev

Time to get the OpenCV 2.4.9 source code:

	>> cd ~
	>> wget http://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.4.9/opencv-2.4.9.zip
	>> unzip opencv-2.4.9.zip
	>> cd opencv-2.4.9

Note: you can always check for newer versions of OpenCv.

Now we have to generate the Makefile by using cmake. In here we can define which parts of OpenCV we want to compile. Since we want to use the viz module, Python, Java, TBB, OpenGL, Qt, work with videos, etc, here is where we need to set that. Just execute the following line at the terminal to create the appropriate Makefile. Note that there are two dots at the end of the line, it is an argument for the cmake program and it means the parent directory (because we are inside the build directory, and we want to refer to the OpenCV directory, which is its parent).

	>> mkdir build
	>> cd build
	>> cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON ..

Check that the above command produces no error and that in particular it reports FFMPEG as YES. If this is not the case you will not be able to read or write videos. Check that Java, Python, TBB, OpenGL, V4L, OpenGL and Qt are all detected correctly.

Make sure you scroll up and check that the modules that are going to be built are these: core flann imgproc highgui features2d calib3d ml video legacy objdetect photo gpu ocl nonfree contrib java python stitching superres ts videostab viz.

If anything is wrong, go back, correct the errors by maybe installing extra packages and then run cmake again.

Now, you are ready to compile and install OpenCV 2.4.9:

	>> make
	>> sudo make install

Now you have to configure OpenCV. First, open the opencv.conf file with the following code:

	>> sudo gedit /etc/ld.so.conf.d/opencv.conf

Add the following line at the end of the file(it may be an empty file, that is ok) and then save it:

	>> /usr/local/lib

Run the following code to configure the library:

	>> sudo ldconfig

Now you have to open another file:

	>> sudo gedit /etc/bash.bashrc

Add these two lines at the end of the file and save it:

	PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig
	export PKG_CONFIG_PATH

Finally, close the console and open a new one, restart the computer or logout and then login again. OpenCV will not work correctly until you do this.

## Installing the stereo_calib_lib repository

First you must download the repository to a directory stereo_calib_lib in your PC, by doing:
	
	>> mkdir stereo_calib_lib
	>> cd stereo_calib_lib
	>> git clone https://github.com/nmoutinho/stereo_calib_lib.git .

You now must link the stereo_calib_lib respository to your project. You can do this by creating a symbolic link to stereo_calib_lib in the root of your project, where your CMakeLists.txt file is located. In the CMakeLists.txt file of your project, add the following line:

	>> add_subdirectory(stereo_calib_lib)

This way you can compile and link the stereo_calib_lib to your project.

## Setting the stereo_calib_lib parameters

The system receives only left and right images as input. To initialize the stereo_calib_lib you have to set the parameters (spherical_multiple_filter_stereo_calib_params) for the two cameras (intrinsic parameters) and define the baseline between the left and right cameras:

 - spherical_multiple_filter_stereo_calib_params

	- baseline: in mm

	- left_cam_resx: width of your left image
	- left_cam_resy: height of your left image
	- left_cam_cx: left image principal point x
	- left_cam_cy: left image principal point y
	- left_cam_fx: left image focal length x
	- left_cam_fy: left image focal length y

	- right_cam_resx: width of your right image
	- right_cam_resy: height of your right image
	- right_cam_cx: right image principal point x
	- right_cam_cy: right image principal point y
	- right_cam_fx: right image focal length x
	- right_cam_fy: right image focal length y

## Single image rectification

If for some reason you need to rectify your images and remove the radial distortion before applying them to the spherical_multiple_filter_stereo_calib, you can use the imagesBase class, as shown in the follwing example.

## Example

This example uses OpenCv:

	#include <opencv/cv.h>
	#include <opencv/highgui.h>
	#include "spherical_multiple_filter_stereo_calib_lib.h"
	#include "images/imagesBase.h"	

	int main(int argc, char * argv[])
	{
	    //this are the intrinsic parameters of your cameras before rectification
	    imagesBase_initial_parameters iip;
	    iip.left_resx = 640;
	    iip.left_resy = 480;
	    iip.left_cx = 337.38258;
	    iip.left_cy = 241.82695;
	    iip.left_fx = 433.16067;
	    iip.left_fy = 433.81054;
	    iip.left_k1  = -0.32797;
	    iip.left_k2  = 0.08380;
	    iip.left_k3  = -0.00068;
	    iip.left_p1  = 0.00098;
	    iip.left_p2  = 0.0;

	    iip.right_resx = 640;
	    iip.right_resy = 480;
	    iip.right_cx = 339.67895;
	    iip.right_cy = 249.41158;
	    iip.right_fx = 442.82627;
	    iip.right_fy = 442.72700;
	    iip.right_k1  = -0.34868;
	    iip.right_k2  = 0.10345;
	    iip.right_k3  = -0.00090;
	    iip.right_p1  = 0.00008;
	    iip.right_p2  = 0.0;

	    imagesBase ib(iip);

	    //capture the first images from your cameras
	    VideoCapture cap1, cap2;
	    cap1.open(0);
	    cap2.open(1);

	    Mat leftraw, rightraw, left, right;
	    cap2 >> right;
	    cap1 >> left;

	    //rectify your images. Get the new intrinsic parameters that should be sent to the stereo calibration system.
	    imagesBase_data ibd = ib.rectify(left, right);
	    Mat kleft = ibd.calibMatLeft;
	    Mat kright = ibd.calibMatRight;
	    int width = iip.left_resx;
	    int height = iip.left_resy;

	    double resize_factor = 2.5;

	    //set the parameters for the stereo calibration system
	    spherical_multiple_filter_stereo_calib_params cscp_general;
	    cscp_general.baseline = 67;//in mm
	    cscp_general.left_cam_resx = width/resize_factor;
	    cscp_general.left_cam_resy = height/resize_factor;
	    cscp_general.left_cam_cx = kleft.at<double>(0,2)/resize_factor;
	    cscp_general.left_cam_cy = kleft.at<double>(1,2)/resize_factor;
	    cscp_general.left_cam_fx = kleft.at<double>(0,0)/resize_factor;
	    cscp_general.left_cam_fy = kleft.at<double>(1,1)/resize_factor;
	    cscp_general.right_cam_resx = width/resize_factor;
	    cscp_general.right_cam_resy = height/resize_factor;
	    cscp_general.right_cam_cx = kright.at<double>(0,2)/resize_factor;
	    cscp_general.right_cam_cy = kright.at<double>(1,2)/resize_factor;
	    cscp_general.right_cam_fx = kright.at<double>(0,0)/resize_factor;
	    cscp_general.right_cam_fy = kright.at<double>(1,1)/resize_factor;

	    spherical_multiple_filter_stereo_calib csc(cscp_general);

	    while(1)
	    {
		cap2 >> right;
		cap1 >> left;

		imagesBase_data ibd = ib.rectify(left, right);

		Mat left_rz, right_rz;

		resize(ibd.rectifiedLeftImage, left_rz, Size(left.cols/resize_factor,left.rows/resize_factor));
		resize(ibd.rectifiedRightImage, right_rz, Size(right.cols/resize_factor,right.rows/resize_factor));

		csc.calibrate(left_rz, right_rz);

		//get the calibrated transformations between the two cameras
		spherical_multiple_filter_stereo_calib_data cscd =  csc.get_calibrated_transformations();

		//obtain and show the disparity map
		spherical_multiple_filter_stereo_disparity_data csdd = csc.get_disparity_map(left_rz, right_rz);
        	imshow("disparity", csdd.disparity_image);
		waitKey(1);

		//show the transformation between the left and right images
		cout << "Transformation from left to right camera: " << cscd.transformation_left_cam_to_right_cam << endl;

	    }

	    return 0;
	}
