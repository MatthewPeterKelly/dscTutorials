%EoM_Double_Pendulum.m

% This script derives the equations of motion for a double pendulum using
% the lagrange equations and the matlab symbolic toolbox.

% Model: Double Pendulum, with a point mass bobs. Theta is measured from the
% -j axis. Gravity points in the -j direction. The pendulum is a fixed
% length, and there is no friction.


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

syms u1 u2 'real'
% u1 == Torque acting on link 1 wrt ground
% u2 == Torque acting on link 2 wrt link 1

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            vector stuff                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

i = sym([1;0;0]);
j = sym([0;1;0]);
k = sym([0;0;1]);

e1 = cos(th1)*i + sin(th1)*j;  %Unit vector along first link
n1 = -sin(th1)*i + cos(th1)*j;  %Unit vector normal to first link

e2 = cos(th2)*i + sin(th2)*j;  %Unit vector along first link
n2 = -sin(th2)*i + cos(th2)*j;  %Unit vector normal to first link

de1 = dth1*n1; %First time derivative of unit vector along first link
dn1 = -dth1*e1;

de2 = dth2*n2; %First time derivative of unit vector along second link
dn2 = -dth2*e2;

dde1 = ddth1*n1 + dth1*dn1;  %Second derivatives
ddn1 = -ddth1*e1 - dth1*de1;

dde2 = ddth2*n2 + dth2*dn2;  %Second derivatives
ddn2 = -ddth2*e2 - dth2*de2;

g1 = d1*e1;  %Position of center of mass of first link
dg1 = d1*de1;  %Velocity of the center of mass of first link
ddg1 = d1*dde1; %Accelerationh of the CoM of the first link

p1 = l1*e1;  %Position of the first joint

g2 = l1*e1 + d2*e2;  %Position of the center of mass of the second link
dg2 = l1*de1 + d2*de2;  %Velocity of the center of mass of the second link
ddg2 = l1*dde1 + d2*dde2;  %Acceleration of the CoM of the second link

p2 = p1 + l2*e2; %Position of the second joint

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                               Energy                                    %
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

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Newton Euler Equations                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Momentum balance for both links about the origin
% = Moment from weight 1 + moment from weight 2 + Control torque 1
Sum_M_O = cross(g1,-m1*g*j) + cross(g2,-m2*g*j) + u1*k;
% = translation mass 1 + translation mass 2
Sum_DH_O = cross(g1,m1*ddg1) + cross(g2,m2*ddg2) + I1*ddth1*k + I2*ddth2*k;

%Momentum balance for second link about central pivot
% moment from weight 2 + control moment
Sum_M_P1 = cross(g2 - p1, - m2*g*j) + u2*k;
% translation mass 2
Sum_DH_P1 = cross(g2 - p1, m2*ddg2) + I2*ddth2*k;

%Collect Equations:
eqn1 = simplify(Sum_M_O - Sum_DH_O);
eqn2 = simplify(Sum_M_P1 - Sum_DH_P1);
EoM = simplify([dot(eqn1,k); dot(eqn2,k)]);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                   mass matrix gymnastics                                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% This is a confusing step - read carefully:
%
% We know that our equations are of the form:
%             EoM = M(q,dq)*qdd + f(q,dq) == 0;
%
% Thus, we can find M(q,dq) by using the jacobian command:

M = jacobian(EoM,[ddth1, ddth2]);

% Now, we want to find f(q,dq). We can do this by substituting in zero for
% the acceleration vector (dqq)

f = subs(EoM,[ddth1, ddth2],sym([0, 0]));


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                               write files                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

writeDoublePendulumDynamics(f,M);
writeDoublePendulumEnergy(T,U);
writeDoublePendulumPosition(p1,p2,g1,g2);
