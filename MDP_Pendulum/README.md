# README  --  Markov Decision Process - Pendulum
This directory contains code for finding the optimal policy for an inverted pendulum, given actuator and state limits. 

## Entry-point scripts:
- **MAIN_solvePendulum.m** Script containing all user-set parameters. It calls the function to build the MDP and then solve for the optimal policy. 

- **TEST_barycentric.m** Script that is used to test the barycentric interpolation functions, which are crucial in the conversion from a continuous to a discrete system.
