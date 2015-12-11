# Robot Arm Trajectory

This directory contains Matlab code to pose and then solve for a minimum-jerk trajectory for a robot arm that satisfies either either boundary-values or node constraints, subject to joint angle and rate constraints.

The problem is posed and solved as a quadratic program, using multi-segment Chebyshev orthogonal collocation for transcription.

