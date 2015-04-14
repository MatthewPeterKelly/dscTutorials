function y = barycentricInterpolate(Y,Idx,W)
% y = barycentricInterpolate(Y,Idx,W)
%
% This function interpolates a point y, based on the data in Y and the
% weights (W) on the verticies Y(Idx)
%
% INPUTS:
%   Y = [N, 1] data set.
%   W = [d+1, n] weight to apply to each index
%   Idx = [d+1, n] linear index corresponding to each weight
%
% OUTPUTS:
%   y = [n,d] = function value at query points
%
% NOTES:
%   --> y = f(x), where x is d-dimensional, and y is scalar
%   --> Y = Y(x1,x2,...,xd)
%   --> n = size(Y);
%   --> y(X(:,i)) = W(i,:)*Y(Idx(:,i));
%
% See Also: barycentricWeights

n = size(Idx,2);
y = zeros(n,1);

for query=1:n
    y(query) = dot(W(:,query),Y(Idx(:,query)));
end

end