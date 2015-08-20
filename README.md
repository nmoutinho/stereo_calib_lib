# stereo_calib_lib
Stereo calibration library for C++ (implemented by Nuno Moutinho)

The stereo_calib_lib calibrates any stereo vision system in an online manner, with or without encoder measurements. The system assumes every stereo system has 6 measurements: Rx(left), Ry(left), Rz(left), Rx(right), Ry(right) and Rz(right).

This system should be left running during operation and provides the calibrated transformation between the two cameras at any time instance. 

# Installation

First you should download the repository by doing:

    git clone https://github.com/nmoutinho/stereo_calib_lib.git

