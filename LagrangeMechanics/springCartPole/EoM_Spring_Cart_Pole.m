%EoM_Spring_Cart_Pole.m

% This script derives the equations of motion for a cart-pole system with
% a spring connecting the cart to the origin, using the lagrange equations 
% and the matlab symbolic toolbox.

% The lagrangian (L) is defined as:
% 
%       L = T - U
%
% where 
%       T = system's kinetic energy
%       U = system's potential energy

% How we go about expressing the equations of motion:
%
%       DL      D  / DL \         * Note that some of those 'D' should be
%       ---  =  -- | -- |           curvy 'D' to represent partial
%       Dq      Dt \ Dq /           derivatives

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        set up variables                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
clc; clear;

syms x dx ddx th dth ddth 'real'; 
% x == horizontal position of the cart
% th == angle of the point-mass pendulum

syms m1 m2 k g l 'real';
% m1 == mass of the cart
% m2 == mass of the pendulum bob
% k = spring constant
% g == acceleration due to gravity
% l = length of the pendulum

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            vector stuff                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

i = sym([1;0]);
j = sym([0;1]);

e = [sin(th); -cos(th)];
n = [cos(th); sin(th)];


de = dth*n; %First time derivative of unit vector along the pendulum
dn = -dth*e;

p1 = x*i;
p2 = p1 + l*e;

dp1 = dx*i;
dp2 = dp1 + l*de;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Lagrangian Definitions                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Kinetic energy:
T = (1/2)*m1*sum(dp1.*dp1) + (1/2)*m2*sum(dp2.*dp2);

%Potential energy:
U = (1/2)*k*x*x + m2*g*sum(p2.*j);

%Lagrangian:
L = T - U; 

%Generalized coordinates:
q = [x, th];
dq = [dx, dth];
ddq = [ddx, ddth];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  evaluate partial derivatives                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%              DL  
%  DL_Dq  ==  ---      Note that 'D' is partial derivative here 
%              Dq
%
DL_Dq = jacobian(L,q')';

%              DL  
%  DL_Ddq  ==  ---      Note that 'D' is partial derivative here 
%              Ddq
%
DL_Ddq = jacobian(L,dq);

%                D  / DL  \         * Note that some of those 'd' should be
% DDL_DtDdq  ==  -- | --  |         curvy 'D' to represent partial
%                Dt \ Ddq /         derivatives
%
% Note the application of the chain rule:  (Quoting Andy Ruina: )
%      d BLAH / dt  =  jacobian(BLAH, [q qdot])*[qdot qddot]'
%
DDL_DtDdq = jacobian(DL_Ddq',[q, dq]) * [dq, ddq]';


%Write out as single equation and simplify:
EoM = DL_Dq - DDL_DtDdq;
EoM = simplify(EoM);


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

f = subs(EoM,ddq,sym([0, 0]));


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                               write files                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

writeSpringCartPoleDynamics(f,M);

