/**
 * Localization Challenge
 * 
 * main.cpp 
 *
 * Initiates program main loop, contains function templates for 
 * localization procedures to be implemented.
 */

#include <stdio.h>
#include <stdlib.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <math.h>
#include <vector>
#include "RobotDefinitions.h"
#include "LocalizationController.h"
#include "main.h"
#include <eigen3/Eigen/Dense>
#include <random>

#define M_PI  3.14159265358979323846 /* pi */

using namespace std;
using namespace Eigen;

// Flag to check for user initiated uncertainty display

bool displayFlag = true;

/**
 *
 * Extended Kalman Filter Global Variables.
 *
 */

vector<FieldLocation> landmarks;

double q;
double alpha1;
double alpha2;
double alpha3;
double alpha4;

MatrixXd A(3,3);
MatrixXd B(3,2);
MatrixXd I(3,3);
MatrixXd H(2,3);
MatrixXd Q(2,2);
MatrixXd R(2,2);
MatrixXd K(3,2);
MatrixXd S(2,2);
MatrixXd xPrevious(3,1);
MatrixXd xPredicted(3,1);
MatrixXd pPrevious(3,3);
MatrixXd pPredicted(3,3);
MatrixXd muN(3,1);
MatrixXd z(2,1);
MatrixXd zState(2,1);
MatrixXd y(3,1);
MatrixXd xUpdate(3,1);
MatrixXd pUpdate(3,3);

// Generating Gaussian Noise models with 0.0 mean

default_random_engine generator;
normal_distribution<double> distribution(0.0,0.2);

/**
 * getRobotPositionEstimate()
 * This function is called by the controller to retrieve the current 
 * robot position estimate. 
 */
void getRobotPositionEstimate(RobotState& estimatePosn)
{
   // TODO: Write your procedures to set the current robot position estimate here
   
   estimatePosn.x = xUpdate(0,0);
   estimatePosn.y = xUpdate(1,0);
   estimatePosn.theta = xUpdate(2,0); 

}

/**
 * motionUpdate()
 * This function is called every time the position of the robot is
 * updated. The argument passed is the relative change in position of the 
 * robot in local robot coordinates (observed by odometry model), which 
 * may be subject to noise (according to motion model parameters).
 */
void motionUpdate(RobotState delta)
{
    // TODO: Write your motion update procedures here

    // Generating Gaussian Noise for delta simulation

    double noiseX = distribution(generator);
    double noiseTH = distribution(generator); 

    /**
     * The following three lines of code is to be used to not used simulated noise alongside delta updates
      
       double dx = delta.x + (noiseX);
       double dy = delta.y;
       double dth = delta.theta + (noiseTH);
     
     */
    
    double dx = delta.x * (1 + noiseX) + 0.00001;            
    double dth = delta.theta * (1 + noiseTH) + 0.00001;
    double th = xPrevious(2,0);
 
    if(th > M_PI)
         th = th - 2 * M_PI;
    else if(th < -M_PI)
         th = th + 2 * M_PI;

    // State transition matrix. By multiplying the state by this and adding control factors, we get a prediction of the state for the next time step

    A << 1, 0, -1 * dx/dth * cos(th) + dx/dth * cos(th + dth), 
         0, 1, -1 * dx/dth * sin(th) + dx/dth * sin(th + dth), 
         0, 0, 1;

    // Control matrix. This is used to define linear equations for any control factors

    B << -1 * sin(th) + sin(th + dth), dx/dth * (sin(th) - sin(th + dth)) + dx * cos(th + dth), 
         cos(th) - cos(th + dth), -1 * dx/dth * (cos(th) - cos(th + dth)) + dx * sin(th + dth),
         0, dth;

    // Estimated process error covariance matrix

    Q << alpha1 * (dx/dth * dx/dth) + alpha2, 0,
         0, alpha3 * (dx/dth * dx/dth) + alpha4;

    xPrevious = xPredicted;
    pPrevious = pPredicted;
    
    // Control vector. This indicates the magnitude of control based on dynamics of the system on the situation

    muN << -1 * dx/dth * sin(th) + dx/dth * sin(th + dth), 
            dx/dth * cos(th) - dx/dth * cos(th + dth), 
            dth;

    // State Prediction and Update

    xPredicted = xPrevious + muN;

    // Covariance Prediction and Update

    pPredicted = A * pPrevious * A.transpose() + B * Q * B.transpose();
    
    xUpdate = xPredicted;
    pUpdate = pPredicted;

    
}

/**
 * sensorUpdate()
 * This function is called every time the robot detects one or more
 * landmarks in its field of view. The argument passed contains all 
 * marker obervations (marker index and position of marker in robot 
 * coordinates) for the current frame.
 */
void sensorUpdate(std::vector<MarkerObservation> observations)
{

    // TODO: Write your sensor update procedures here
    
    for(int i = 0; i < observations.size(); i++)
    {

        // Landmark Location coordinates
        double mx = landmarks[observations[i].markerIndex].x;
        double my = landmarks[observations[i].markerIndex].y;

        // Current robot coordinates
        double px = xPredicted(0,0);
        double py = xPredicted(1,0);
        double pth = xPredicted(2,0);

        // Distance Squared from Landmark coordinates to Robot Coordinates

        q = (mx - px) * (mx - px) + 
            (my - py) * (my - py) ; 
       
        // Measurement vector matrix. This contains the real-world measurement we received in this time step

        z << observations[i].distance, 
             observations[i].orientation; 

        // Estimation matrix based on Landmarks

        zState << sqrt(q), 
                  atan2(my - py, mx - px) - pth;
                   
        // Observation matrix. By multiplying a state vector by H to translate it to a measurement vector

        H << -1 * ((mx - px)/sqrt(q)), -1 * ((my - py)/sqrt(q)), 0, 
                  ((my - py)/q), -1 * ((mx - px)/q), -1;

        // Innovation covariance matrix. This is where we compare real error against prediction
        
        S = H * pPredicted * H.transpose() + R;

        // Kalman Gain. This is where we moderate the prediction
        
        K = pPredicted * H.transpose() * S.inverse();

        // State Prediction and Update. New estimate of where the robot is
            
        xPredicted = xPredicted + K * (z - zState);

        // Covariance Prediction and Update. New estimate of error
    
        pPredicted = (I - K * H) * pPredicted;
    }

}

/**
 * myinit()
 * Initialization function that takes as input the initial 
 * robot state (position and orientation), and the locations
 * of each landmark (global x,y coordinates).
 */
void myinit(RobotState robotState, RobotParams robotParams, 
            FieldLocation markerLocations[NUM_LANDMARKS])
{
    // TODO: Write your initialization procedures here
       
    I << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    
    // Estimated measurement error covariance matrix

    R << robotParams.sensor_noise_distance * robotParams.sensor_noise_distance, 0, 
         0, robotParams.sensor_noise_orientation * robotParams.sensor_noise_orientation;
    
    // Odometry Noise

    alpha1 = robotParams.odom_noise_translation_from_translation;
    alpha2 = robotParams.odom_noise_translation_from_rotation;
    alpha3 = robotParams.odom_noise_rotation_from_translation;
    alpha4 = robotParams.odom_noise_rotation_from_rotation;

    xPrevious << robotState.x, robotState.y, robotState.theta;
    xPredicted << robotState.x, robotState.y, robotState.theta;
    xUpdate << robotState.x, robotState.y, robotState.theta;
    pPrevious << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    
    for(int i = 0; i < NUM_LANDMARKS; i++) 
        landmarks.push_back(markerLocations[i]);

}

/**
 * mydisplay()
 * This function is called whenever the display is updated. The controller
 * will draw the estimated robot position after this function returns.
 */
void mydisplay()
{
    // TODO: Write your drawing procedures here 
    //       (e.g., robot position uncertainty representation)
    
    if(displayFlag)
    {
    // Example drawing procedure
    int pixelX, pixelY;
    double degInRad;
    
    const int NUM_POINTS = 360;
    double POINT_SPREAD_X = pUpdate(0,0);
    double POINT_SPREAD_Y = pUpdate(1,1);
        
    // Draw cyan colored points to represent uncertainty ellipse on field
    glBegin(GL_POINTS);
    glColor3f(0.0, 1.0, 1.0);
    for(int i=0; i<NUM_POINTS; i++)
    {
        degInRad = i;
        global2pixel(xUpdate(0,0) + (i * POINT_SPREAD_X * cos(degInRad)), xUpdate(1,0) + (i * POINT_SPREAD_Y * sin(degInRad)), pixelX, pixelY);
        glVertex2i(pixelX, pixelY);
    }
    
    glEnd();
    
    }
}

/**
 * mykeyboard()
 * This function is called whenever a keyboard key is pressed, after
 * the controller has processed the input. It receives the ASCII value 
 * of the key that was pressed.
 *
 * Return value: 1 if window re-draw requested, 0 otherwise
 */
int mykeyboard(unsigned char key)
{
    // TODO: (Optional) Write your keyboard input handling procedures here

    // Detecting key "D" or "d" to initiate or deactivate display of uncertainty ellipse
	if(key == 'd' || key == 'D')
    {
        displayFlag = !displayFlag;
        cout << "Uncertainty Visualization: ";
        if(displayFlag == 1)
            cout << "ON" << endl;
        else
            cout << "OFF" << endl;
    }
    
	return 0;
}


/**
 * Main entrypoint for the program.
 */
int main (int argc, char ** argv)
{
    // Initialize world, sets initial robot position
    // calls myinit() before returning
    runMainLoop(argc, argv);
    return 0;
}

