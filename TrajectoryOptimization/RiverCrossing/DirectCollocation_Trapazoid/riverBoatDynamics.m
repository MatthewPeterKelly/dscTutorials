function dz = riverBoatDynamics(z,u,param)
% dz = riverBoatDynamics(z,u)
%
% This function copmutes the dynamics of a boat as it crosses a river. The
% water in the river is flowing with a parabolic speed profile. The boat
% can travel in any direction at a constant speed with respect to the
% water.
%
% INPUTS:
%   z = [2, n] = [x;y] = state matrix
%   u = [1, n] = control
%
% OUTPUTS:
%   dz = [2, n] = dz/dt = first time derivative of the state
%
% NOTES:
%   The river is flowing in the positive x direction.
%
%

%%% Unpack the state
% x = z(1,:);   % dz does not depend on position along the river
y = z(2,:);

%%% River water speed (positive x direction)
v = -param.maxWaterSpeed*y.*(y-param.riverWidth);

%%% Dynamics
dx = v + param.boatSpeed*cos(u);
dy = param.boatSpeed*sin(u);

%%% Pack up state:
dz = [dx;dy];

end