function [C, Ceq] = multipleShooting2(z,pack,dyn)
%
% This function enforces the dynamics of the cart-pole system
% using 4th-order runge kutta:
%
%
% INPUTS:
% 	z = [N,1] = vector of decision variables
% 	pack = struct with matrix dimensions
% 	dyn = struct with physical parameters
%
% OUTPUTS:
% 	C = []
% 	Ceq = column vector of defects. Trajectory is feasible iff norm(Ceq)->0
%
% NOTES:
%  	[t,x,u] = unPackDecVar(z,pack);
%   size(t) == [1,1];
% 	size(x) == [nState, nSegment+1]
% 	size(u) == [nControl, nSubStep*nSegment+1]	
%
% * The trajectory is represented with piecewise-linear control
% 	and piece-wise quartic state (implicityly). 		
%

[t,x,u] = unPackDecVar(z,pack);
nx = size(x,2);
nu = size(u,2);

nSubStep = (nu-1)/(nx-1);
dt = t/(nu-1);

xLow = x(:,1:(nx-1));  %Each segment starts here
xUpp = x(:,2:nx);   %And goes here

% Time-step along each trajectory segment in parallel
xStar = xLow;
uIdx = 0:nSubStep:(nu-1);
for i=1:nSubStep
	uStarLow = u(:,i + uIdx);
	uStarUpp = u(:,i+1 + uIdx);
	uStarMid = 0.5*(uStarLow+uStarUpp);
    k1 = cartPoleDynamics(xStar,uStarLow,dyn);
    k2 = cartPoleDynamics(xStar+0.5*k1,uStarMid,dyn);
    k3 = cartPoleDynamics(xStar+0.5*k2,uStarMid,dyn);
    k4 = cartPoleDynamics(xStar+k3,uStarUpp,dyn);
    xStar = xStar + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end

% Defect constraint:
defect = xStar-xUpp;

% Pack up and return:
C = [];
Ceq = reshape(defect,numel(defect),1);

end