%EoM_Single_Pendulum.m

% This script derives the equations of motion for a single pendulum using
% the lagrange equations and the matlab symbolic toolbox.

% Model: Single Pendulum, with a point mass bob. Theta is measured from the
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
%       dL      d  / dL \         * Note that some of those 'd' should be
%       ---  =  -- | -- |           curvy 'd' to represent partial
%       dth     dt \ dw /           derivatives

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        set up variables                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

syms th w a 'real'; 
% th == angle from -j axis
% w  == angular rate w = d(th)/dt
% a  == angular acceleration a = d(w)/dt

% Note that in the standard notation we would have:
%    th == "q"  --  generalized coordinate
%    w  == "q dot"
%    a  == "q double dot"

syms m g l 'real';
% m == (point) mass of the pendulum bob
% g == acceleration due to gravity
% l == length of the pendulum

T = (1/2)*m*(l^2)*(w^2);  % kinetic energy of the pendulum
U = m*g*l*(1-cos(th));  % potential energy of the pendulum
L = T - U;  % lagrangian

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  evaluate partial derivatives                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%              dL  
%  dL_dth  ==  ---      Note that 'd' is partial derivative here 
%              dth
%
dL_dth = jacobian(L,th);

%              dL  
%  dL_dw   ==  ---      Note that 'd' is partial derivative here 
%              dw
%
dL_dw = jacobian(L,w);

%               d  / dL \         * Note that some of those 'd' should be
% ddL_dtdw  ==  -- | -- |           curvy 'd' to represent partial
%               dt \ dw /           derivatives
%
% Note the application of the chain rule:
%
ddL_dtdw = jacobian(dL_dw,[th, w]) * [w, a]';

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  solve equations and write files                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Equations of motion:
eqn = ddL_dtdw - dL_dth;

% Solve for acceleration:
soln = solve(eqn,a);

% Write dynamics function:
filename = 'singlePendulumDynamics.m';
matlabFunction(soln,'file',filename,'vars',{th,g,l},'output',{'ddth'});

% Write energy function:
filename = 'singlePendulumEnergy.m';
totalEnergy = T+U;
energy = [totalEnergy;T;U];
matlabFunction(totalEnergy,T,U,'file',filename,'vars',{th,w,m,g,l},'output',{'energy','kinetic','potential'});



