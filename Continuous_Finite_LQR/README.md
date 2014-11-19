#README.md

This directory contains a tutorial for using matlab to stabilize a trajectory of a non-linear system, using a finite-horizon continuous-time Linear Quadratic Regulator (LQR).

## Key Files:

### MAIN_trajectory:
Entry-point function for the primary demo, using the full non-linear plant
- Plots vector field for non-linear system
- Generate a nominal trajectory
- Approximate trajectory using polynomial curve fitting
- Solve LQR along the trajectory
- Approximate gains using polynomial curve fitting
- Compare the gains obtained from finite vs infinite horizon lqr
- Run simulations to demonstrate controller's stabilizing properties

### trajectoryLqr:
Solves the finite-horizon continuous-time LQR problem for a time-varying plant (stabilize a non-linear trajectory)

### Derive_EoM:
Derives the dynamics equations and writes function files for the non-linear plant. 

### MAIN_linearPlant:
A quick little demo to find the finite-horizon continuous-time LQR gains for a linear plant, and then comparing the solutions to the infinite-horizon version, generated using Matlab's `lqr` command.

### finiteLqr:
Solves the finite-horizon continuous-time LQR problem for a linear time-invariant plant.
