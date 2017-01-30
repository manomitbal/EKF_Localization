# EKF_Localization
Extended Kalman Filter based State Estimation of a differential drive robot.

Feature Based Localization

Problem Statement

This challenge provides a feature based localization engine with a computer simulated 2D
soccer field, as inspired by the RoboCup competition. The objective of this challenge is to
procure and accurately estimate the position and orientation [x, y, θ] T of an autonomous
mobile robot.

Requirements

C++ 11: The Makefile as provided has been accommodated to be compiled with C++ 11
standard, depicted by ”-std=c++11” under CXXFLAGS. This is mainly required for the
Gaussian noise generator used.

Eigen Library: Run the following commands to install the eigen library. The INCLUDE
flag in the Makefile takes care of the library call during compiling.

sudo apt-get update

sudo apt-get install libeigen3-dev

This library mostly takes care of all the matrix functions including Inverse and Trans-
pose.

Glut Library: The Glut library is primarily used for GUI and visualization function-
ality. If you have a graphics card, it is required to be linked under the LIBRARIES tag in
the Makefile. In my case, ”-L/usr/lib/nvidia-367” was added.

sudo apt-get install freeglut3-dev

Run command ”make” in the folder with the core files. To run the executable, type:

./localization test < input file >
