# README.md  --  Trajectory Optimization - Cannon Example

## Overview:
This directory contains Matlab code for solving the optimal trajectory for aiming a cannon with non-negligable quadratic drag on the cannon ball. The optimization problem is transcribed using three different methods: 
    - Single Shooting
    - Multiple Shooting
    - Collocation (Hermite-Simpson)

## Entrypoint Scripts
    - MAIN_cannon.m - runs the main tutorial, including animations
    - TEST_cannonDynamics.m - simple forward simulation of dynamics with event detection for the ground
    - TEST_cannonFeasibility.m - batch run forward simulations to understand relationship between initial launch speed and range.

## Key Functions:
	- cannon_singleShooting.m
	- cannon_collocation.m
	- cannon_multipleShooting.m

 ## Implementation Details
 
 In all cases I use the same exact initialization, based off of a forward simulation with event detection. This is not the best initialization for each method, but it provides some consistency between transcription methods and is sufficient.

I used Matlab's FMINCON optimization algorithm, although similar results could be obtained with other programs, such as SNOPT or IPOPT.

 The _single shooting_ method is implemented using the standard 4th-order Runge-Kutta integration scheme for the simulation, and the total time is included as a decision variable. 

 The _multiple shooting_ method is also implemented using 4th-order Runge-Kutta, but with several intermediate states included as decision variables. This makes the method more robust than single shooting.

 The _collocation_ method is implemented using Hermite-Simpson quadrature. This is just one of many possible schemes.
