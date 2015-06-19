function [tIdx,xIdx,uIdx] = getIndex(pack)

% This function returns the indices to extract decision variables for use
% in linear constraints to be passed to fmincon.

nDecVar = 2 + pack.nState(1)*pack.nState(2) + ...
    pack.nControl(1)*pack.nControl(2);
[tIdx,xIdx,uIdx] = unPackDecVar(1:nDecVar,pack);

end