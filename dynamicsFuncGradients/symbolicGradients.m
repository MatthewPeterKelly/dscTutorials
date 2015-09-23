function [Az, Bz] = symbolicGradients(A,b,q,dq,u)
% [Az, Bz] = symbolicGradients(A,b,q,dq,u)
%
% This function computes the symbolic gradients of the dynamics of a system
% in mass matrix form:
%
% A(q,dq,u)*ddq = b(q,dq,u);
%
% q = [n, 1] = configuration 
% dq = [n, 1] = rates
% ddq = [n,1] = accelerations
% u = [m,1] = system inputs
%
% Let:
%   z = [q;dq;u] 
%
% Then Az = dA/dz, which has been reshaped to a column vector:
%   Az = [n*n*(2*n+m),1];
%   bz = [n*(2*n+m),1];
%
%

% DecisionVars:
z = [q;dq;u];

n = length(q);
m = length(u);
l = 2*n+m;

% Compute jacobian of mass matrix and generalized forces:
Az = reshape(jacobian(reshape(A,n*n,1),z),n*n*l,1);
Bz = reshape(jacobian(b,z),n*l,1);

end