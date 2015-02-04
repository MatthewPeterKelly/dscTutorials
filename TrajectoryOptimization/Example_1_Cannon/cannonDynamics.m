function dz = cannonDynamics(~,z,c)
% function dz = cannonDynamics(~,z,c)
%
% This function computes the dynamics of a cannon ball travelling through
% the air with quadratic drag
%
% INPUTS:
%   ~ = [1 x N] time vector (not used)
%   z = [4 x N] state vectory
%       z(1,:) = x = horizontal position
%       z(2,:) = y = vertical position
%       z(3,:) = dx = horizontal speed
%       z(4,:) = dy = vertical speed
%   c = quadratic drag coefficient
%
% OUTPUTS:
%   dz = d(x)/dt = time-derivative of the state
%
% NOTES:
%   - Assume unit gravity and mass
%   - Assume that cannon ball is a point mass  (no rotational inertia)
%

dz = zeros(size(z));

%First-order form (derivative of position states is velocity states)
dz(1:2,:) = z(3:4,:);

% Compute the speed (used for drag force calculation)
dx = z(3,:);
dy = z(4,:);
v = sqrt(dx.*dx + dy.*dy); 

% Compute the force vectors
fx = -c*dx.*v;
fy = -c*dy.*v - 1;  %Assume unit gravity

% Record the accelerations (derivative of velocity states)
dz(3,:) = fx;  %Assume unit point-mass
dz(4,:) = fy;  %Assume unit point-mass

end