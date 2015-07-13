# Piece-wise Polynomial (pwPoly)

A collection of functions for working with piece-wise polynomial representations of functions in Matlab.

- pwPoly2 = quadratic spline. useful if function value is known at mid-points
- pwPoly3 = cubic spline. Wrappers for matlab's pchip, spline, ppval, and related commands
- pwPoly4 = quartic spline. Curiosity, not too useful. first-order continuous at knot points, with function value defined at the mid-point
- pwPoly5 = 5th-order spline, second-order smooth at knot points. These functions are the most developed in the library.

A few demonstrations (DEMO_*.m) are provided to show how to use the program

