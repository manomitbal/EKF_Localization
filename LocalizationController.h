/**
 * Localization Challenge
 * 
 * LocalizationController.h
 *
 * Declares functions available to user for interfacing
 * with controller, including setting the current robot
 * position estimate.
 */
#ifndef _LOCALIZATION_CONTROLLER_H_
#define _LOCALIZATION_CONTROLLER_H_

#include "RobotDefinitions.h"

// Initializes robot state and marker locations, 
// sets up OpenGL window management, then calls user-defined 
// myinit() function before starting main controller loop
void runMainLoop(int argc, char** argv);

// Convert from pixel to global coordinates
void pixel2global(int pixelX, int pixelY, double& globalX, double& globalY);

// Convert from global to pixel coordinates
void global2pixel(double globalX, double globalY, int& pixelX, int& pixelY);

#endif
