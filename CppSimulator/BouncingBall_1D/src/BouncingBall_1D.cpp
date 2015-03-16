//============================================================================
// Name        : BouncingBall_1D.cpp
// Author      : Matthew P. Kelly
// Description : Simulation of simple bouncing ball (1D)
//============================================================================

#include "Ball.h"

#include <stdio.h>
#include <stdlib.h>

using namespace std;

int main() {
	Ball* ball = new Ball();
	FILE* dataFile;
	dataFile = fopen("data.csv","w");

	int bounceCount = 0;
	bool bounce;
	ball->printState(dataFile);
	while (bounceCount < 4){
		bounce = ball->timeStep();
		ball->printState(dataFile);
		if (bounce){
			bounceCount++;
		}
	}

	fclose(dataFile);
	// system("./plotData.py");  // Calls python plotting script
	return 0;
}
