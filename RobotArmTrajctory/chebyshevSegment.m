function [A,b,H,t,D] = chebyshevSegment(n,d,dqMax)
% 
%
% This function computes the constraints and objective function for a
% single segment of a trajectory for the robot arm problem.
%
% 
%
% OUTPUTS:
%   Ax <= b  % Joint rate constraints
%   x'*H*x   % Quadratic cost matrix
%   t = time grid points
%

% Derivatives
D = chebyshevDifferentiationMatrix(n,d);   %Rates
DD = D*D;    %Accelerations
DDD = DD*D;   %Jerk

% Chebyshev grid and quadrature weights:
[t,w] = chebyshevPoints(n,d);

%%%% Quadratic Program:
%   min 0.5*x'*H*x + f'*x   subject to:  A*x <= b
%    x                                   Aeq*x == beq
%                                        LB <= x <=UB

%%%% Dynamics:
% u == DDD*x;   %Equality Constraint - expressed directly in cost function

%%%% Joint Rate Limits:
% -dqMax <= D*x <= dqMax
A = [D;-D];
b = dqMax*ones(2*n,1);

%%%% Objective Function:
% min  w*(u^2) = x'*(DDD'*W*DDD);  W = diag(w);
W = diag(w);
H = 0.5*(DDD'*W*DDD);   %Minimum Jerk

end