function dz = cartPoleDynamicsHumanReadable(z,u,p)
% dz = cartPoleDynamicsHumanReadable(z,u,p)
%
% This function computes the first-order dynamics of the cart-pole.
%
% INPUTS:
%   z = [4, n] = [x;q;dx;dq] = state of the system
%   u = [1, n] = horizontal force applied to the cart
%   p = parameter struct
%       .g = gravity
%       .m1 = cart mass
%       .m2 = pole mass
%       .l = pendulum length
% OUTPUTS:
%   dz = dz/dt = time derivative of state
%
%

% x = z(1,:);   %Cart position (Not used in dynamics)
q = z(2,:);   % pendulum (pole) angle, measure from gravity vector
dx = z(3,:);  % cart velocity
dq = z(4,:);  %pendulum angle rate

% Unpack the physical parameters
l = p.l;  %Pendulum length
m1 = p.m1; % cart mass
m2 = p.m2; % pole mass
g = p.g;  %Gravity acceleration

% The following expressions were derived using the symbolic math toolbox,
% in the script: Derive_cartPole.m
% Typically I use the automatically generated code, but I've written out
% the code in a more readable way here for tutorial purposes.

mEff = m1 + m2 - m2.*cos(q).^2;
centr = dq.^2.*l.*m2.*sin(q);
ddx = (u + centr + g.*m2.*cos(q).*sin(q))./mEff;
ddq = -(u.*cos(q) + (m1 + m2)*g*sin(q) + centr.*cos(q))./(l.*mEff);

dz = [dx;dq;ddx;ddq];

end