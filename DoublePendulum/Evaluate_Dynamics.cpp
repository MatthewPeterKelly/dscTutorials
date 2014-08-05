// General include statements
#include "mex.h"
#include "math.h"
#include "matrix.h"
#include <typeinfo>

//Main dynamics function
#include "dynamics.cpp"

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
/*

Matlab Call:  dX = Evaluate_Dynamics(X, U, Cpp_Params);

 *  IMPORTANT - U is a [6xN] Matrix!!!
 
Torque = a vector of output torques

Xin =  [Nstate x Nruns] a matlab matrix, where each column is the state of the robot
Cpp_Params = pointer to a mex vector of parameters
*/

	// Initialize the inputs 
	double *U;   // Input matrix [Ninput x Nruns]  
	double *X;   //set of states of interest [Nstate x Nruns]
	const mxArray* params; //Store the parameter vector here

	mwIndex Nstate;   //What is the dimension of the state?
	mwIndex Nruns;   //How many parallel runs?
	mwIndex Ninput;    // How many input channels are there?

	// Initialize the outputs 
	double *dX;       // Output matrix of derivatives

	//Temp variables:
	double *x;    // store the input state for each run...
	double *u;    // Store the input for each run...
	double *dx;   // Store the output derivative for each run...

	unsigned stateDim;    // Counter for going along the state dimension (N)
	unsigned inputDim;    // Counter for going along the input dimension 
	unsigned trialNum;    // Counter for going through each trial

	// Populate the input dimensions - look at Xin
	Nstate = (mwIndex)mxGetM(prhs[0]);   //Find the number of states  (matlab rows)
	Nruns = (mwIndex)mxGetN(prhs[0]);   //Find the number of runs    (Matlab columns)
	Ninput =(mwIndex)mxGetM(prhs[1]);    // Get the number of inputs (matlab rows)

	// Get the inputs 
	//Matlab Call:  Torque = Evaluate_Controller(Xin, Cpp_Params);
	X = mxGetPr(prhs[0]);      // Get the pointer to the state
	U = mxGetPr(prhs[1]);      //Get the pointer to the input
	params = prhs[2]; // parameters for RHS of ODE  //Get the pointer to the parameter vector

	/* Get the outputs */
	plhs[0] = mxCreateDoubleMatrix(Nstate, Nruns, mxREAL);   //For now, only handles scalar control output
	dX= mxGetPr(plhs[0]);

	//Get the parameters:
	#include "GetParameters.h"

	//initialization
	x = new double[Nstate];   // Dynamic allocation for state vector
	u = new double[Ninput];   // Dynamic allocation for state vector
	dx = new double[Nstate];   // Dynamic allocation for state vector

	// Useful things now:
	for(trialNum=0; trialNum<Nruns; trialNum++){

		//Store the current state
		for(stateDim=0; stateDim<Nstate; stateDim++){
			x[stateDim] = X[Nstate*(trialNum) + stateDim];
		}

		//Store the current input
		for(inputDim=0; inputDim<Ninput; inputDim++){
			u[inputDim] = U[Ninput*(trialNum) + inputDim];
		}

		//Call the dynamics
		dynamics(u, x, P_dynamics, dx);

		// Return the derivatives:
		for(stateDim=0; stateDim<Nstate; stateDim++){
			dX[Nstate*(trialNum) + stateDim] = dx[stateDim];
		}

	} // for trialNum

	//deallocate memory
	delete [] x;    //Deallocate dynamic memory
	delete [] u;    //Deallocate dynamic memory
	delete [] dx;    //Deallocate dynamic memory
}