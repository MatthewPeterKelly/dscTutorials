%EoM_Spring_Cart_Pole.m

% This script derives the equations of motion for rolling disk with a
% pendulum hanging from the center. Derived using the newton-euler
% equations and the matlab symbolic toolbox.

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        set up variables                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
clc; clear;

syms phi dphi ddphi th dth ddth 'real'; 
% phi == angle of th disk
% th == angle of the point-mass pendulum

syms m1 m2 I g r l 'real';
% m1 == mass of the disk
% m2 == mass of the bob
% I = moment of inertia of the disk
% r = radius of the disk
% g == acceleration due to gravity
% l = length of the pendulum

syms T 'real'
% T = constraint force - to be removed

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            vector stuff                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

i = sym([1;0;0]);
j = sym([0;1;0]);
k = sym([0;0;1]);

e = i*sin(th) -j*cos(th);
n = i*cos(th) + j*sin(th);

de = dth*n; %First time derivative of unit vector along the pendulum
dn = -dth*e;

dde = ddth*n - dth*dth*e;
ddn = -ddth*e - dth*dth*n;

p1 = -r*phi*i;
p2 = p1 + l*e;

dp1 = -r*dphi*i;
dp2 = dp1 + l*de;

ddp1 = -r*ddphi*i;
ddp2 = ddp1 + l*dde;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Newton-Euler Equations                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Angular momentum about the rolling contact point:
sumTorque = cross(r*j,-T*e);
sumMomentum = I*ddphi*k + cross(r*j,m1*ddp2);
eqn1 = dot(k,sumTorque-sumMomentum);

%Linear momentum for the pendulum bob:
sumForces = T*e - m2*g*j;
Accel = m2*ddp2;
eqn2 = simplify(dot(e,sumForces-Accel));
eqn3 = simplify(dot(n,sumForces-Accel));

%Get rid of the constraint force:
Tsolve = solve(eqn2,T);
EqnA = subs(eqn1,T,Tsolve);
EqnB = subs(eqn3,T,Tsolve);

%Express equations of motion in vector form:
EoM = simplify([EqnA; EqnB]);

%Express accelerations in a vector:
ddq = [ddphi; ddth];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                   mass matrix gymnastics                                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% This is a confusing step - read carefully:
%
% We know that our equations are of the form:
%             EoM = M(q,dq)*qdd + f(q,dq) == 0;
%
% Thus, we can find M(q,dq) by using the jacobian command:

M = jacobian(EoM,ddq);

% Now, we want to find f(q,dq). We can do this by substituting in zero for
% the acceleration vector (dqq)

f = subs(EoM,ddq,sym([0; 0]));


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                               write files                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

writeRollingCartPoleDynamics(f,M);

