/*
 * Ball.cpp
 *
 *  Created on: Mar 14, 2015
 *      Author: Matthew P. Kelly
 */

#ifndef BALL_H_
#define BALL_H_

#include <stdio.h>

class Ball {
private:
	double t;  // time
	double p;  // position
	double v;  // velocity
	double k;  // coefficient of restitution
	double dt; // time step for integration method
	double g;  // gravity
	double quadSolve(double a, double b, double c, double tLow, double tUpp);
public:
	Ball();
	virtual ~Ball();
	void printState(FILE* file);
	void setTime(double time);
	void setTimeStep(double timeStep);
	void setPosition(double pos);
	void setVelocity(double vel);
	bool timeStep(); // Runs a single time step of physics
};

#endif /* BALL_H_ */
