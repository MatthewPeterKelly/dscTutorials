function [t,x,u] = unPackDecVar(z,pack)
% function [t,x,u] = unPackDecVar(z,pack)
%
% This function unpacks the decision variables for
% trajectory optimization into the duration (t), 
% state (x), and control (u) matricies
%
% See Also: PACKDECVAR

ns = pack.nState;  % ns = size(x)
nc = pack.nControl; % nc = size(u)

is = 1:(ns(1)*ns(2));
ic = 1:(nc(1)*nc(2));

t = z(1);
x = reshape(z(1+is),ns(1),ns(2));
u = reshape(z(1+is(end)+ic, nc(1),nc(2)));

end