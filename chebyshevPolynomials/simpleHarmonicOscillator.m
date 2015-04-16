function dz = simpleHarmonicOscillator(z,P)
%dz = simpleHarmonicOscillator(z,P)
%
% FUNCTION:
%   This function is used to compute the dynamics of a simple harmonic
%   oscillator. This is used for testing ode solvers with a simple system.
%
% INPUTS:
%   t = [1 x Nt] input time vector. (NOT USED)
%   z = [Ns x Nt] input state vector.
%   P = struct of parameteres with fields:
%       m = massls
%       c = damping constant
%       k = spring constant
%
% OUTPUTS:
%   dz = the derivative of z with respect to time.
%

%Physical parameters
m = P.m;
c = P.c;
k = P.k;

%States
x = z(1,:);
v = z(2,:);

%Solution:
dz = zeros(size(z));
dz(1,:) = v;
dz(2,:) = -(c/m)*v - (k/m)*x;

end