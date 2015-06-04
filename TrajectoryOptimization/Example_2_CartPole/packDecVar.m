function [z,pack] = packDecVar(t,x,u)
% [z,pack] = packDecVar(t,x,u)
%
% This function collapses the durtion (t), state (x)
% and control (u) matricies into a single vector
%
% See Also: UNPACKDECVAR

pack.nState = size(x);
pack.nControl = size(u);

z = [t; reshape(x,numel(x),1); reshape(u,numel(u),1)];

end
