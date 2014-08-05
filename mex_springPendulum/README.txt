README.txt  --  SpringPendulum  --  Tutorial for Matlab MEX files

This code demonstrates how to use a simple MEX interface to call a simulation in C++ from Matlab. This system is a point-mass pendulum with an axial spring connecting the point mass to the hinge. There is a sinusoidal forcing torque on the system. Integration is done using 4th-order Runge Kutta, implemented in C++. Matlab is only used for setting the simulation parameters, compiling the C++ code, and then plotting the results.

RUN:  MAIN.m