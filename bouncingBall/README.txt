README  --  Bouncing Ball
UPDATED --  November 15, 2013

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Run MAIN.m to see a simulation of a bouncing ball

Edit Set_Parameters.m to change simulation parameters

Edit groundHeight.m to change the shape of the ground

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


These files are used to run a simulation of a ball (point mass) bouncing
over a smooth curving surface. There is quadratic drag on the ball while
it is in the air, and each collision is modelled as instantaneous and 
described using a coefficient of restitution.

MAIN.m does the simulation by executing the following loop:
    Loop:
       ode45 integration until collision
       impact map calculation
       reset initialization for ode45
    Break: if out of time or too many bounces

The last part of MAIN.m then uses deval to interpolate the ode45 
gridpoints and then stitches the results into a single large data set.
Once a single data set has been created the plotting and animation
routines are called.


~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The ground is modelled as a sum of three sine waves, but this could be
replaced by any function with a continuous first derivative.


~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Under certain conditions, it is possible for the simulation to allow
the ball to pass through the ground. This can happen for two reasons:

1) If the if two successive gridpoints are above the ground, then 
the event detection assumes that the ball did not pass through the 
ground. This is a bad assumption if there is a steep hill and the ball 
is moving horizontally through it. 

2) If the ball velocity after the collision is very small, it can 
sometimes case the event detection to fail.