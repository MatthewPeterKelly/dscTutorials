% Derive_EoM.m
%
% This script derives the equations of motion for the simple cart-pole
%
% MODEL:
%   Point-mass cart with a pendulum hanging from it. There are two
%   actuators: a horizontal force applied to the cart and a torque applied
%   to the base of the pendulum. Assume a friction-less track.
%
% NAMES:
%   x = horizontal position of the cart
%   q = angle of the pendulum, measured from stable configuration
%   
%   dx = dx/dt = horizontal velocity of the cart
%
%   F = horizontal force applied to the cart
%   T = torque applied to the base of the cart
%
%   m1 = cart mass
%   m2 = pendulum mass
%   g = gravity
%   l = pendulum length
%
clc; clear;

syms x q dx dq ddx ddq 'real' % States and Rates
syms F T 'real'  %Actuators
syms m1 m2 g l 'real'  %Physical parameters

% inertial reference frame:
i = sym([1;0]);
j = sym([0;1]);

% unit vector along the pendulum:
e = cos(q)*(-j) + sin(q)*(i);

% state vector:
z = [x;q;dx;dq];
dz = [dx;dq;ddx;ddq];

% kinematics of the point-mass on the pendulum:
p = i*x + l*e;
dp = jacobian(p,z)*dz;
ddp = jacobian(dp,z)*dz;

% kinematics of the center of mass:
M = m1+m2;
G = (m1*i*x + m2*p)/M;
dG = jacobian(G,z)*dz;
ddG = jacobian(dG,z)*dz;

% horizontal force balance:
eqn1 = dot(F-M*ddG, i);

% torque balance about base of the pendulum:
cross2d = @(a,b)( a(1)*b(2) - a(2)*b(1) );  %dot(cross(a,b),k);
sumTorque = T + cross2d(l*e, -m2*g*j);  %joint torque + gravity torque
angMomDot = cross2d(l*e, m2*ddp);  %Angular momentum about joint
eqn2 = sumTorque - angMomDot;  

% collect equations and write in form:
%   massMatrix*accel = genForces
[massMatrix,genForce] = equationsToMatrix([eqn1;eqn2],[ddx;ddq]);
accel = massMatrix\genForce;
soln.ddx = simplify(accel(1));
soln.ddq = simplify(accel(2));

% write out the forward dynamics:
matlabFunction(soln.ddx, soln.ddq,...
    'file','autoGen_cartPoleDynamics.m',...
    'vars',{q,dq,F,T,m1,m2,g,l},...
    'outputs',{'ddx','ddq'});

% inverse dynamics:
soln.F = simplify(solve(eqn1,F));
soln.T = simplify(solve(eqn2,T));
matlabFunction(soln.F, soln.T,...
    'file','autoGen_cartPoleInvDyn.m',...
    'vars',{q,dq,ddq,ddx,m1,m2,g,l},...
    'outputs',{'F','T'});

