Function Smoothing via Polynomials

===========================================

Updated: October 8, 2013

Written by Matthew Kelly
Cornell University

Functions:
 - SmoothAbs
 - SmoothBnd
 - SmoothRamp

Each function is a smooth version of a simple function.
The user can specify the degree of smoothness, up to 8th order smooth at the transitions. The user can also specify a degree of 0, causing the function to return the original non-smooth function.

Each function has a testing script that shows the effect
that different values of the smoothing parameter have.

There are a few other functions in this directory:

 - SolveCoeff -- symbolically finds the coefficients of a polynomial to satisfy a given set of constraints. It will return an error if the constraints are not well posed. It is used to find the coefficients of the polynomials that are used for smoothing.

 - writeSmoothRamp -- a function to write SmoothRamp

 - writeSmoothAbs --  a function to write SmoothAbs

