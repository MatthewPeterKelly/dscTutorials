function dz = cartPoleDynamics(z,u,p)
% dz = cartPoleDynamics(z,u,p)
%
% This function computes the dynamics for a simple cart-pole: a point-mass
% cart that travels on a friction-less horizontal track. A pendulum hangs
% from the cart. There are two actuators: a horizontal force that pushes on
% the cart, and a torque acting on the base of the pendulum.
%
% INPUTS:
%   z = [4, n] = state vector = [x;q;dx;dq]
%   u = [1, n] = actuation vector = F = force on cart
%   p = struct of parameters:
%       .m1 = cart mass
%       .m2 = pendulum point-mass
%       .g = gravity
%       .l = length of the pendulum
%
% OUTPUTS:
%   dz = [4, n] = derivative of the state vector
%

% x = z(1,:);
q = z(2,:);
dx = z(3,:);
dq = z(4,:);

F = u(1,:);
T = zeros(1,length(F));

[ddx,ddq] = autoGen_cartPoleDynamics(q,dq,F,T,p.m1,p.m2,p.g,p.l);

dz = [dx;dq;ddx;ddq];

end


