function [tSpan,x,u] = unPackDecVar(z,dim)
% function [tSpan,x,u] = unPackDecVar(z,dim)
%
% This function unpacks the decision variables for
% trajectory optimization into the duration (t), 
% state (x), and control (u) matricies
%
% INPUTS:
%   z = [2+ns*ms+nc*nc, 1] = vector of decision variables
%   dim = struct with the size of the state and control matricies
%       .nState = size(x);
%       .nControl = size(u);
%
% OUTPUTS:
%   tSpan = [1, 2] = [t0, tF] = time span
%   x = [ns, ms] = state matrix = [dimension in state space, grid point]
%   u = [nc, mc] = control matrix = [dimension in control space, grid point]
%
% See Also: PACKDECVAR

ns = dim.nState;  % ns = size(x)
nc = dim.nControl; % nc = size(u)

is = 1:(ns(1)*ns(2));
ic = 1:(nc(1)*nc(2));

tSpan = [z(1), z(2)];
x = reshape(z(2+is),ns(1),ns(2));
u = reshape(z(2+is(end)+ic), nc(1),nc(2));

end