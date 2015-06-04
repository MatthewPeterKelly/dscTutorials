function [C, Ceq] = multipleShooting1(z,pack,dyn)
%
% This function enforces the dynamics of the cart-pole system
% using euler integration and multiple shooting.
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
% 	size(u) == [nControl, nSubStep*nSegment]	
%
% * The trajectory is represented with piecewise-constant control
% 	and piece-wise linear state. 		
%


[t,x,u] = unPackDecVar(z,pack);
nx = size(x,2);
nu = size(u,2);

nSubStep = nu/(nx-1);
dt = t/nu;

xLow = x(:,1:(nx-1));  %Each segment starts here
xUpp = x(:,2:nx);   %And goes here

% Time-step along each trajectory segment in parallel
xStar = xLow;
uIdx = 0:nSubStep:(nu-1);
for i=1:nSubStep
	uStar = u(:,i + uIdx);
    xStar = xStar + dt*cartPoleDynamics(xStar,uStar,dyn);
end

% Defect constraint:
defect = xStar-xUpp;

% Pack up and return:
C = [];
Ceq = reshape(defect,numel(defect),1);

end
