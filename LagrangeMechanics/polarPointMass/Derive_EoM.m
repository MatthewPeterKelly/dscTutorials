% Derive EoM.m
%
% This script derives the equations of motion for a point-mass pendulum
% with an axial force actuator, rather than a rigid link.
%

syms th dth ddth r dr ddr 'real'  % State variables
syms m g 'real'  % Parameters
syms u 'real'  % Actuator

i = sym([1;0]);
j = sym([0;1]);

e = -sin(th)*i + cos(th)*j;   %Unit vector along pendulm axis
n = -cos(th)*i - sin(th)*j;   %Unit vector orthogonal to e

de = dth*n;
dn = -dth*e;

dde = ddth*n - dth*dth*e;
ddn = -ddth*e - dth*dth*n;

p = r*e;  % Position of the point-mass
dp = dr*e + r*de;
ddp = (ddr*e + dr*de) + (dr*de + r*dde);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                              Force Balance                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

sumForces = u*e - m*g*j;
massAccel = m*ddp;

EoM = sumForces - massAccel;  % == zero


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                   mass matrix gymnastics                                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% This is a confusing step - read carefully:
%
% We know that our equations are of the form:   (q = state vector)
%             EoM = M(q,dq)*qdd + f(q,dq) == 0;
%
% Thus, we can find M(q,dq) by using the jacobian command:

M = jacobian(EoM,[ddr; ddth]);

% Now, we want to find f(q,dq). We can do this by substituting in zero for
% the acceleration vector (ddq)

f = subs(EoM,[ddr; ddth],sym([0; 0]));

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                   Write Dynamics function                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

accel = simplify(-M\f);
write_dynamics(accel)

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Energy Stuff                                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

energy = simplify(m*g*dot(p,j) + 0.5*m*dot(dp,dp));
energyRate = simplify(m*g*dot(dp,j) + m*dot(ddp,dp));

%Get rid of acceleration terms in energyRate:
energyRate = simplify(subs(energyRate,ddr,accel(1)));
energyRate = simplify(subs(energyRate,ddth,accel(2)));





