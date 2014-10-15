%EoM_Double_Pendulum.m

% This script derives the equations of motion for a double pendulum using
% the lagrange equations and the matlab symbolic toolbox.

% Model: Double Pendulum, with a point mass bobs. Theta is measured from the
% -j axis. Gravity points in the -j direction. The pendulum is a fixed
% length, and there is no friction.

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

syms th1 th2 dth1 dth2 ddth1 ddth2 'real'; 
% th1 == absolute angle of the first link, measured from +x axis
% th2 == absolute angle of the second link, measured from +x axis
% dth1 == first time derivative of th1
% dth2 == first time derivative of th2
% ddth1 == second time derivative of th1
% ddth2 == second time derivative of th2

syms m1 m2 g l1 l2  I1 I2 d1 d2 'real';
% m1, m2 == mass of the pendulum links
% g == acceleration due to gravity
% l1, l2 == length of the pendulum's first and second links
% I1, I2 == moment of inertias of the first and second links
% d1, d2 == distance between link's parent joint and CoM

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            vector stuff                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

e1 = [cos(th1); sin(th1)];  %Unit vector along first link
n1 = [-sin(th1); cos(th1)];  %Unit vector normal to first link

e2 = [cos(th2); sin(th2)];  %Unit vector along first link
n2 = [-sin(th2); cos(th2)];  %Unit vector normal to first link

de1 = dth1*n1; %First time derivative of unit vector along first link
dn1 = -dth1*e1;

de2 = dth2*n2; %First time derivative of unit vector along second link
dn2 = -dth2*e2;

g1 = d1*e1;  %Position of center of mass of first link
dg1 = d1*de1;  %Velocity of the center of mass of first link

p1 = l1*e1;  %Position of the first joint

g2 = l1*e1 + d2*e2;  %Position of the center of mass of the second link
dg2 = l1*de1 + d2*de2;  %Velocity of the center of mass of the second link

p2 = p1 + l2*e2; %Position of the second joint


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Lagrangian Definitions                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Kinetic energy:
T1 = (1/2)*m1*dot(dg1,dg1) + (1/2)*I1*dth1*dth1;  %First link
T2 = (1/2)*m2*dot(dg2,dg2) + (1/2)*I2*dth2*dth2;  %Second link
T = T1 + T2;

%Potential energy:
yIndex = 2;
U1 = m1*g*g1(yIndex);
U2 = m2*g*g2(yIndex);
U = U1 + U2;

%Lagrangian:
L = T - U; 

%Generalized coordinates:
q = [th1, th2];
dq = [dth1, dth2];
ddq = [ddth1, ddth2];


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

writeDoublePendulumDynamics(f,M);
writeDoublePendulumEnergy(T,U);
writeDoublePendulumPosition(p1,p2,g1,g2);
