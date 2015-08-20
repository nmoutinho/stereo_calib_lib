# stereo_calib_lib
Stereo calibration library for C++ (implemented by Nuno Moutinho)

The stereo_calib_lib calibrates any stereo vision system in an online manner, with or without encoder measurements. The system assumes every stereo system has 6 measurements: Rx(left), Ry(left), Rz(left), Rx(right), Ry(right) and Rz(right).

This system should be left running during operation and provides the calibrated transformation between the two cameras at any time instance. 

# Requirements

To use stereo_calib_lib you need the follwing libraries:

 - gcc
 - Cmake >= 2.8
 - OpenCv >= 2.4.9

# Configure the system (on Ubuntu 12.04 LTS or 14.04 LTS) - installing gcc, Cmake and OpenCv 2.4.9

OK, so the first step is to make sure that everything in the system is updated and upgraded. Open the terminal and write this:

	>> sudo apt-get update
	>> sudo apt-get upgrade

Now, you need to install many dependencies, such as support for reading and writing image files, drawing on the screen, some needed tools, other libraries, etc... This step is very easy, you only need to write the following command in the Terminal:

	>> sudo apt-get install build-essential libgtk2.0-dev libjpeg-dev libtiff4-dev libjasper-dev libopenexr-dev cmake python-dev python-numpy python-tk libtbb-dev libeigen3-dev yasm libfaac-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev libqt4-dev libqt4-opengl-dev sphinx-common texlive-latex-extra libv4l-dev libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev default-jdk ant libvtk5-qt4-dev

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

# Installation

First you have to download the repository to any _FOLDER_, by doing:

	>> cd _FOLDER_
	>> git clone https://github.com/nmoutinho/stereo_calib_lib.git

You need to link the stereo_calib_lib respository to your project. You can do this by creating a symbolic link to _FOLDER_/stereo_calib_lib in the root of your project, where your CMakeLists.txt file is located. In the CMakeLists.txt file of your project, add the following line:

	>> add_subdirectory(stereo_calib_lib)

This way you can compile and link the stereo_calib_lib to your project.

# Example

This example uses OpenCv 
