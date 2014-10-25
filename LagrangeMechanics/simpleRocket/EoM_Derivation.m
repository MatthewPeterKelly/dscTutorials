%EoM_Derivation  --  Simple Rocket
%
% This script is used to generate the equations of motion for a simple
% rocket: ~slender rod with constant mass, two force actuators, no air
% resistance, inverse-square gravity law.
%

syms m G mPlanet l d  %Parameters
% m = rocket mass (constant)
% G = gravitational constant
% l = rocket length
% d = engine offset

syms u1 u2  %Inputs (engines)

syms r th1 th2  %State
syms dr dth1 dth2  %State Rate
syms ddr ddth1 ddth2  %State Acceleration
% r = distance from center of planet to rocket CoM
% th1 = angle of position of the rocket, in the world frame
% th2 = orientation of the rocket, in the world frame

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Unit Vectors                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

i = sym([1;0]);
j = sym([0;1]);

e1 = -sin(th1)*i + cos(th1)*j;   %Unit vector from center of planet to rocket CoM
n1 = -cos(th1)*i - sin(th1)*j;   %orthogonal to e1;

de1 = dth1*n1;
dn1 = -dth1*e1;

dde1 = ddth1*n1 - dth1*dth1*e1;
ddn1 = -ddth1*e1 - dth1*dth1*n1;

e2 = -sin(th2)*i + cos(th2)*j;   %Unit vector from engine to tip of rocket
n2 = -cos(th2)*i - sin(th2)*j;   %orthogonal to e2;

de2 = dth2*n2;
dn2 = -dth2*e2;

dde2 = ddth2*n2 - dth2*dth2*e2;
ddn2 = -ddth2*e2 - dth2*dth2*n2;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Position Vectors                                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

p = r*e1;
dp = dr*e1 + r*de1;
ddp = (ddr*e1 + dr*de1) + (dr*de1 + r*dde1);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Newton Euler Equations                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

w = G*mPlanet*m/(r*r);    %Weight force on CoM of rocket

%Sum of forces:
sumForces = -w*e1 + (u1 + u2)*e2;
linAccel = m*ddp;
eqn1 = simplify(dot(i,sumForces-linAccel));
eqn2 = simplify(dot(j,sumForces-linAccel));

%Sum of torques
sumTorques = d*(u1-u2);
angAccel = (1/12)*m*l*l*ddth2;
eqn3 = simplify(sumTorques-angAccel);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Solve Equations                                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

eqn = [eqn1; eqn2; eqn3];
ddq = [ddr; ddth1; ddth2];
M = jacobian(eqn,ddq);
f = subs(eqn,ddq,[0;0;0]);
dyn = -M\f;

writeDynamics(dyn);   %Write the dynamics function


