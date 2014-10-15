%EoM_SimpleHarmonicOscillator.m

% This script derives the equations of motion for a simple harmonic oscillator
% using the lagrange equations and the matlab symbolic toolbox.

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
%       dx      dt \ dv /           derivatives

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        set up variables                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

syms x v a 'real'; 
% x == position 
% v  == velocity
% a  == acceleration

% Note that in the standard notation we would have:
%    x == "q"  --  generalized coordinate
%    v  == "q dot"
%    a  == "q double dot"

syms m k 'real';
% m == (point) mass of the pendulum bob
% k == spring constant

T = (1/2)*m*v^2;  % kinetic energy of the pendulum
U = (1/2)*k*x^2;  % potential energy of the pendulum
L = T - U;  % lagrangian

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  evaluate partial derivatives                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%              dL  
%  dL_dth  ==  ---      Note that 'd' is partial derivative here 
%              dth
%
dL_dx = jacobian(L,x);

%              dL  
%  dL_dv   ==  ---      Note that 'd' is partial derivative here 
%              dw
%
dL_dv = jacobian(L,v);

%               d  / dL \         * Note that some of those 'd' should be
% ddL_dtdv  ==  -- | -- |           curvy 'd' to represent partial
%               dt \ dv /           derivatives
%
% Note the application of the chain rule:
%
ddL_dtdv = jacobian(dL_dv,[x, v]) * [v, a]';

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  solve equations and write files                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Equations of motion:
eqn = ddL_dtdv - dL_dx;

% Solve for acceleration:
soln = solve(eqn,a);

%Print solution to screen:
disp(' ')
disp('Equations of motion for simple harmonic oscillator: ');
disp('dx = v');
disp(['dv = ' vectorize(char(soln))]);


