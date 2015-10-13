# Robot Arm Trajectory

This directory contains Matlab code to pose and then solve for a minimum-jerk trajectory for a robot arm that satisfies either either boundary-values or node constraints, subject to joint angle and rate constraints.

## Dependencies

This code uses chebyshev orthogonal collocation. The helper-functions that I use can be found in 
        dscTutorials/chebyshevPolynomials/