function [ddq, ddqz] = dynamicsNumeric(q,dq,u)
% [ddq, ddqz] = dynamicsNumeric(q,dq,u)
%
% This function computes the dynamics and the gradient of the dynamics for
% the system by numerically solving linear systems
%
% INPUTS:
%   q = [nq, nt] = matrix of configurations
%   dq = [nq, nt] = matrix of rates
%   u = ,nu, nt] = matrix of inputs
%
% OUTPUTS:
%   ddq = [nq,nt] = matrix of accelerations
%   ddqz = [nq, (nq+nq+nu), nt] = gradients of ddq wrt [q;dq;u];
%

empty = zeros(size(q(1,:)));  %Stupid hack for vectorization
[A,B,Az,Bz] = autoGen_dynamicsNumeric(q(1,:),q(2,:),dq(1,:),dq(2,:),u(1,:),u(2,:),u(3,:),empty);
nx = size(q,1);
nu = size(u,1);
nz = 2*nx + nu;
[ddq, ddqz] = computeGradientOfBackSlash(A,B,Az,Bz,nx,nz);

end