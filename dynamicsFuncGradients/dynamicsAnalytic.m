function [ddq, ddqz]  = dynamicsAnalytic(q,dq,u)
% [ddq, ddqz]  = dynamicsAnalytic(q,dq,u)
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
[ddq,ddqz] = autoGen_dynamicsAnalytic(q(1,:),q(2,:),dq(1,:),dq(2,:),u(1,:),u(2,:),u(3,:),empty);

[nq, nt] = size(q);
nu = size(u,1);

ddqz = reshape(ddqz, nq, (nq+nq+nu), nt);

end