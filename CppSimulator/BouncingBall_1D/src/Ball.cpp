/*
 * Ball.cpp
 *
 *  Created on: Mar 14, 2015
 *      Author: Matthew P. Kelly
 */

#include "Ball.h"
#include <cmath>

#define smallNum 1e-8 // Numerical tolerance around zero

Ball::Ball() {
	t = 0.0;  // time
 	p = 1.0;  // position
 	v = 0.0;  // velocity
 	k = 0.85;  // coefficient of restitution
 	dt = 0.01; // time step for integration method
 	g = 9.81;  // gravity
}

Ball::~Ball() { }

void Ball::printState(FILE* file){
	fprintf(file,"%6.6f, %6.6f, %6.6f \n",t,p,v);
}

void Ball::setTime(double time){
	t = time;
}

void Ball::setTimeStep(double timeStep){
	if (timeStep > 1e-6){
		dt = timeStep;
	} else {
		dt = smallNum;
	}
}

void Ball::setPosition(double pos){
	if (pos > smallNum) {
		p = pos;
	} else {
		p = smallNum;
	}
}

void Ball::setVelocity(double vel){
	v = vel;
}

bool Ball::timeStep(){
	double tNext, pNext, vNext;
	bool collision = false;

	// Symplectic Euler Integration
	tNext = t + dt;
	vNext = v - dt*g;
	pNext = p + dt*vNext;

	// Check for collisions with the ground:
	if (pNext < 0){ // Then collision
		// Solve the system:
		// vStar = v - dtStar*g;
		// pStar = 0 = p + dtStar*vStar;
		// --> 0 = p + dtStar*(v-dtStar*g);
		double dtStar = quadSolve(-g,v,p,0,dt);
		double vStar = v - dtStar*g;
		tNext = t + dtStar;
		pNext = p + dtStar*vStar;
		vNext = -k*vStar;
		collision = true;
	}

	// Update the state:
	t = tNext;
	p = pNext;
	v = vNext;

	return collision;

}

double Ball::quadSolve(double a, double b, double c, double tLow, double tUpp){
	// Solves for the roots of: 0 = a*t*t + b*t + c;
	// numerically stable method
	// Returns the first root that is in the domain [tLow, tUpp]
	double d = b*b - 4*a*c;
	if (d>0) {
		double z;
		if (b>0) {
			z = -0.5*(b+sqrt(d));
		} else {
			z = -0.5*(b-sqrt(d));
		}
		double t1 = z/a;
		double t2 = c/z;
		bool valid1 = (t1>tLow) && (t1<tUpp);
		bool valid2 = (t2>tLow) && (t2<tUpp);
		if (valid1 && valid2){
			return fmin(t1,t2);
		} else if (valid1 && !valid2){
			return t1;
		} else if (!valid1 && valid2){
			return t2;
		} else {
			return tLow;
		}
	} else { // No real root - return lower bound
		return tLow;
	}

}

