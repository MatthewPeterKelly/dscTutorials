smoothVersionExp

Updated: October 25, 2013

Written by Matthew Kelly
Cornell University

Functions:
    SmoothAbs
    SmoothAbsFancy
    SmoothBnd
    SmoothRamp
    SmoothMax

Each function is a smooth version of a simple function.
In each case, the output is full continuous.

In some cases, very small values of alpha can cause an 
overflow. If this occurs, then the functions will return
the non-smooth version of the function.

Each function has a testing script that shows the effect
that different values of the smoothing parameter have.