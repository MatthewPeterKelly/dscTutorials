# README  --  Lagrance Mechanics

This directory contains matlab code for deriving the equations of motion of a few standard (conservative) mechanical systems. 

For each system there is a script to derive the equations of motion. This script is titled: 
EoM_System_Name.m  

The equation of motion script will automatically generate dynamics and energy functions for this sytem: 
systemNameDynamics.m
systemNameEnergy.m

There is a simulation driver script for each system as well, titled:
MAIN_systemName.m

The system driver script will use ode45 to run a simulation, and then plot or animate the results. In all cases it will call:
systemNameRhs.m  (an interface between ode45 and the dynamics function)
