% Derive_EoM.m
%
% Planar Orbit, Falling Weight
%
% Written by Matthew Kelly
% May 19, 2015
%
% Problem Description:
%   Suppose that you have two point-masses that are connected by a
%   mass-less inextensible string. One mass is able to travel around
%   horizontal planar surface. The second mass is hanging from the point
%   where the string passes through a small hole in the surface.
%
% Problem Statement:
%   Are there stable non-circular solutions to the motion of the first
%   mass?
%
% Method:
%   Build a simulator and see what happens!
%
%


%%%% Define variables:
syms x y dx dy ddx ddy 'real' %State variables
syms g m1 m2 l 'real' %Parameters

%%%%% State Vector:
s = [x;y;dx;dy];
ds = [dx;dy;ddx;ddy];

%%%% Unit Vectors:
i = sym([1;0]);
j = sym([0;1]);

%%%% Kinematics:
p1 = x*i + y*j;
dp1 = dx*i + dy*j;
ddp1 = ddx*i + ddy*j;

r = sqrt(x*x+y*y);  %Length of string above the table
dr = simplify(jacobian(r,s)*ds);  %Chain rule to get rate of change
ddr = simplify(jacobian(dr,s)*ds);

h = r - l;   %Height of the second mass
dh = dr - l;
ddh = ddr - l;

%%%% Compute Lagrangian
KE = ...   %Kinetic energy
    0.5*m1*dot(dp1,dp1) + ...  %Sliding mass on surface
    0.5*m2*dh*dh; %Hanging mass

PE = ... %Potential energy
    m2*g*h;  %Hanging mass

L = KE-PE;


%%%% Lagrange Equations

DL_Dq = jacobian(L,[x;y])';
DL_Ddq = jacobian(L,[dx;dy])';
DDL_DtDdq = jacobian(DL_Ddq,s)*ds;

eqns = simplify(DL_Dq - DDL_DtDdq);

%%%% Extract Acceleration equations:
[MM,ff] = equationsToMatrix(eqns,[ddx;ddy]);
soln = simplify(MM\ff);


%%%% Write function files:
matlabFunction(soln(1),soln(2),'file','autoGen_dynamics.m',...
    'vars',{x,y,dx,dy,m1,m2,g},...
    'outputs',{'ddx','ddy'});



