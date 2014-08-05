Fancy Double Pendulum

====================================

Matthew Kelly
December 12, 2013
Cornell University

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Run: Falling_Simulation/MAIN_Falling_Simulation.m

-> Runs a simulation of the double pendulum falling through space while various contacts are turning on and off randomly. The legs act like springs by default. 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This code is designed to test a new biped locomotion model that I'm working on. The biped is modelled as a double pendulum in space, with a point mass at each foot, and a point mass at the hip. There are several actuators available: ankle torques, hip torque, and leg axial force. The dynamics are solved for four phases of motion:
    Single Stance One - Foot One is fixed, Foot Two is free
    Single Stance Two - Foot Two is fixed, Foot One is free
    Double Stance - Both fixed
    Flight - Both feet are free

The equations of motion for all dynamics functions are derived symbolically and then written to function files automatically in Derive_Equations_of_Motion. The entry-point function is Derive_EoM.m, which calls all of the other functions.

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

MAIN_Falling_Simulation.m was written to test the various automatically generated functions. It randomly generates a series of phase transitions, each of which has a random time. The default mode is forces the actuators to behave as if the system was a spring double pendulum. When in this mode, the system should conserve energy, which provides a useful check on the equations of motion.

It is also possible to turn off the spring mode, and then each actuator has a built in PD controller (gains can be adjusted using Falling_Simulation/setupController.m). 
