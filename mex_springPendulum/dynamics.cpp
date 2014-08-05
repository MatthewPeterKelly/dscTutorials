void dynamics( double X[], double dX[], int Nstate , 
			   double t,   double Params[]         ){
/*
FUNCTION: This function computes the dynamics of a forced spring pendulum

INPUT:
	X = a vector with Nstate elements, that contains the input state.
		for now, X = [x,y,dx, dy]
	dX = a vector with Nstate elements, that contains the derivative of the input state
		with respect to time
	Nstate = the number of elements in X and dX
	t = a scalar value for the current time
	Params = a vector of input parameters, detailed below

OUTPUT:
	dX is an array that is passed by reference. This function only updates dX.
*/

		double m, k, c, g, lo, amp, w;      //scalar parameters names
		double x,y,v,z;    //dynamical states of the system
		double L;   // Length of the spring
		double tau;  // Torque produced by the actuator
		double T;   // Tension Force
		double F;   // Tangential force
		double er[2], et[2];  //unit vectors
		const double PI=3.141592653589793;

		/* Set up the parameters for the dynamics   */
		m = Params[0];	// (kg) mass
		k = Params[1];	// (N/m) spring constant
		c = Params[2];  //  (Ns/m) viscous damping
		g = Params[3];	// (m/s^2) gravitational acceleration
		lo = Params[4];	 // (m) spring rest length
		amp = Params[5];  // (Nm) forcing amplitude
		w = Params[6];  // (Hz) forcing frequency

		/* Evaluate the forcing function */
		w = w*2*PI;   // Convert to useful units
		tau = amp*sin(w*t);

		/* Break out the states */
		x = X[0];  // Position x
		y = X[1];  // Position y
		v = X[2];  // Velocity x
		z = X[3];  // Velocity y

		/* Solve the dynamics in pieces */
		L = sqrt(x*x+y*y);
		T = k*(L-lo);
		F = tau/L;
		er[0] = x/L;    er[1] = y/L;
		et[0] = y/L;    et[1] = -x/L;

		dX[0] = v;
		dX[1] = z;
		dX[2] =      (T/m)*(-er[0]) + (F/m)*(et[0]) - (c/m)*v;  // acceleration in the x direction
		dX[3] = -g + (T/m)*(-er[1]) + (F/m)*(et[1]) - (c/m)*z;  // acceleration in the xy direction

}