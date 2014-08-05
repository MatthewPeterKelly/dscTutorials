// General include statements
#include "mex.h"
#include "math.h"
#include "matrix.h"
#include <typeinfo>


//Dynamics function
#include "dynamics.cpp"  

// Integration subroutines
// #include "eulerMethod.cpp"  
#include "rungeKutta.cpp"


void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
/*
FUNCTION:
	This function is used to integrate a dynamical system forward in time

INPUTS:
	t = a matlab row vector with two elements: [start time, end time];
	x0 = a matlab column vector of the dynamical state at the first time step
	n = a scalar integer that is the number of time steps to use
	Params = a matlab row vector of parameter values - see definitions in dynamics()

OUTPUTS:
	t = a matlab row vector of size (1,Ntime)
	x = a matlab matrix of size (Nstates, Ntime) that contains the state of the system
		at each time in the input vector t

NOTES:
	1)	As of now, this code does NOT do error checking. Poor form on my part. If you do not
		enter the parameter vector to be the correct length, it will cause Matlab to crash...
*/

	/* Initialize the inputs */
	double *Tspan;   //matlab vector [start time, end time];
	double *x0;    // Store the initial conditions here [Matlab Col Vector] = [C++  (1 x N)]


	int Ntime;

	double *Params;  //Store the parameter vector here


	mwSignedIndex Nstate;   //How big is the input?

	/* Initialize the outputs */
	mxArray *x_output;   //Output Array  -- Results go here
	mxArray *t_output;   //Output Array  -- Results go here
	double *t;  //Store the time vector [Matlab Row Vector] = [C++  (M x 1)]
	double *x;    // Here is the output for the system [C++ (M x N)]
	// M = time steps
	// N = state index

	int stateDim;    // Counter for going along the state dimension (N)

	/* Get the number of states*/
	Nstate = (mwSignedIndex)mxGetM(prhs[1]);   //Find the number of states



	/* Get the inputs */
	Tspan = mxGetPr(prhs[0]);   // This command gets the pointer to a real valued array the time span
	x0 = mxGetPr(prhs[1]);      // Get the pointer to the initial conditions


 Ntime = mxGetScalar(prhs[2]);



	Params = mxGetPr(prhs[3]);  //Get the pointer to the parameter vector

	/* Get the outputs */
		t_output = mxCreateDoubleMatrix(1, Ntime, mxREAL);
		plhs[0] = t_output;
		t = mxGetPr(plhs[0]);
		
		x_output = mxCreateDoubleMatrix(Nstate, Ntime, mxREAL);
		plhs[1] = x_output;
		x = mxGetPr(plhs[1]);


							/* Print to matlab */
	mexPrintf("Number of Time Steps =  %d \n",Ntime);
    mexPrintf("Number of States = %d \n",Nstate);
	//mexPrintf(typeid(Ntime).name());

	/* Store the start and end time */
		t[0] = Tspan[0];    // Start time
		t[Ntime-1] = Tspan[1];   // End time

	/* Store the initial conditions */
	for(stateDim=0; stateDim<Nstate; stateDim++){
		x[stateDim] = x0[stateDim];
	}
	

	/* Do the integration */
	rungeKutta(t, x, Ntime, Nstate, Params);

}  
