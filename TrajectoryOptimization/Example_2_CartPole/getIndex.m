function [tIdx,xIdx,uIdx] = getIndex(pack)

% This function returns the indices to extract decision variables for use
% in linear constraints to be passed to fmincon.

nDecVar = pack.nState(1)*pack.nState(2) + ...
    pack.nControl(1)*pack.nControl(2) + 1;
[tIdx,x,u] = unPackDecVar(1:nDecVar,pack);

xIdx.init = x(:,1);
xIdx.final = x(:,end);
xIdx.x = x;
uIdx.u = u;

end