README for Q-Exam Tractor Trailer truck control problem

Problem Statement: Find a controller for a simple model of a tractor trailer truck that can drive backwards and forwards along a curving road. The number of sensors is less than the number of states, and the states are noisy.

Solution: Use an extended Kalman filter to produce a state estimate as the truck is driving. At each time use a discrete linear quadratic regulator to fidn the optimal gains for the linearized system (about the current configuration).

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Run MAIN.M to get a simulation and plots.

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Equations of motion are derived using the Matlab symbolic toolbox, and this derivation can be found in Derive_EoM.m
