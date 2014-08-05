void rungeKutta( double t[], double x[],
			      int Ntime, int Nstate , 
			      double Params[]         ){
/* 
FUNCTION:
	This function integrates the system using the dynamics function
	forward in time via 4th order Runge-Kutta integration
INPUTS:
	t = an array of length Ntime that stores the times that the integration should return
	x = an array of length Ntime*Nstate that stores the dynamical state at each time step
	Ntime = the length of the t array
	Nstate = the number of dynamical states
	Params = a parameter array that is defined in the dynamics function
OUTPUTS:
	This function implicitly returns a populated version of x
*/

	/* misc variables */
		int timeStep;    // Counter for going along the time dimension (M)
		int stateDim;    // Counter for going along the state dimension (N)
		double dt, timeNow;     //Time Step and current time

	/* Initialize the working variables */
		double *X1;    // a vector with N elements (dynamical state at current time step)
		double *X2;    // These store the vectors of the state at various points in the step
		double *X3;
		double *X4;
	
		double *k1;   // a vector with N elements (time derivative of the dynamical state at current time step)
		double *k2;   //k1 through k4 are the derivatives at different points in the time step
		double *k3;
		double *k4;
	
		double *Xnew;   // store the new state here

	/* Dynamic array allocation */
		X1 = new double[Nstate];
		X2 = new double[Nstate];
		X3 = new double[Nstate];
		X4 = new double[Nstate];
	
		k1 = new double[Nstate];
		k2 = new double[Nstate];
		k3 = new double[Nstate];
		k4 = new double[Nstate];
	
		Xnew = new double[Nstate];

	/* IMPORTANT!!!
	MATLAB    [N x M]
	C++       [M x N]
	*/
	

	dt = (t[Ntime-1]-t[0])/(Ntime-1);    //Time Step


	/* Integrate forward in time */
	for(timeStep=1; timeStep<Ntime; timeStep++){
		
		// get the current time information
		timeNow = timeStep*dt;			//Current time


		/* Runge-Kutta part 1 */
			// read in the current state
			for(stateDim=0; stateDim<Nstate; stateDim++){ 
				X1[stateDim] = x[Nstate*(timeStep-1)+stateDim];
			}
			// define the state derivatives
			dynamics(X1, k1, Nstate, timeNow, Params);

		/* Runge-Kutta part 2 */
			// define the second intermediate state
			for(stateDim=0; stateDim<Nstate; stateDim++){ 
				X2[stateDim] = X1[stateDim] + 0.5*dt*k1[stateDim];
			}
			// define the state derivative at the second partial step
			dynamics(X2, k2, Nstate, timeNow+0.5*dt, Params);

		/* Runge-Kutta part 3 */
			// define the third intermediate state
			for(stateDim=0; stateDim<Nstate; stateDim++){ 
				X3[stateDim] = X1[stateDim] + 0.5*dt*k2[stateDim];
			}
			// define the state derivative at the second partial step
			dynamics(X3, k3, Nstate, timeNow+0.5*dt, Params);
		
		/* Runge-Kutta part 4 */
			// define the last intermediate state
			for(stateDim=0; stateDim<Nstate; stateDim++){ 
				X4[stateDim] = X1[stateDim] + dt*k3[stateDim];
			}
			// define the state derivative at the second partial step
			dynamics(X4, k4, Nstate, timeNow+dt, Params);

		// Now integrate the whole step
		for(stateDim=0; stateDim<Nstate; stateDim++){ 
			Xnew[stateDim] = X1[stateDim] + (dt/6)*(k1[stateDim] + 2*k2[stateDim] + 2*k3[stateDim] + k4[stateDim]);
		}
		
		//Write the next time to the output vector
		t[timeStep] = timeNow;

		// write the next state to the output matrix
		for(stateDim=0; stateDim<Nstate; stateDim++){ 
			x[Nstate*(timeStep)+stateDim] = Xnew[stateDim];
		}

	}

	/* Deallocate dynamic arrays */
		delete [] X1;
		delete [] X2;
		delete [] X3;
		delete [] X4;
	
		delete [] k1;
		delete [] k2;
		delete [] k3;
		delete [] k4;
	
		delete [] Xnew;

}