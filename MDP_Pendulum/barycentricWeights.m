function [W,Idx] = barycentricWeights(X,xBnd,N)
% [W,Idx] = barycentricWeights(X,xBnd,N)
%
% This function computes the weights and indicies such that
%
% INPUTS:
%   X = [d , n] = n points of interest, each is d-dimensional
%   xBnd = [d , 2] = [min,max] along each dimension
%   N = [1, d] = number of grid points along each dimension
%
% OUTPUTS:
%   W = [d+1, n] weight to apply to each index
%   Idx = [d+1, n] linear index corresponding to each weight
%
% NOTES:
%   --> y = f(x), where x is d-dimensional, and y is scalar
%   --> Y = Y(x1,x2,...,xd)
%   --> n = size(Y);
%   --> y(X(:,i)) = dot(W(:,i)*Y(Idx(:,i));
%
% ASSUME:
%   --> all data is uniformly spaced:
%       {xi} = linspace(xBnd(i,1), xBnd(i,2), n(i));
%
% REFERENCE:
% 
% Based on paper: "Multidimensional Triangulation and Interpolation for
% Reinforcement Learning" by Scott Davies, NIPS 1996
%
% See Also: barycentricInterpolate

[d, n] = size(X);
W = zeros(d+1,n);
Idx = zeros(d+1,n);
for i=1:n
    [W(:,i),Idx(:,i)] = barycentricWeightsCore(X(:,i),xBnd,N);
end

end

function [w,idx] = barycentricWeightsCore(x,xBnd,N)
% [w,idx] = barycentricWeights(x,xBnd,n)
%
% This function computes the weights and indicies such that y = Y(idx)*w,
% where Y is some data, stored on a length(N) dimensional grid.
%
% INPUTS:
%   x = [d , 1] point of interest
%   xBnd = [d , 2] = [min,max] along each dimension
%   N = [1, d] = number of grid points along each dimension
%
% OUTPUTS:
%   w = [d+1,1] weight to apply to each index
%   idx = [d+1,1] linear index corresponding to each weight
%
% NOTES:
%   --> y = f(x), where x is d-dimensional, and y is scalar
%   --> Y = Y(x1,x2,...,xd)
%   --> n = size(Y);
%   --> y = dot(Y(idx),w);
%
% ASSUME:
%   --> all data is uniformly spaced:
%       {xi} = linspace(xBnd(i,1), xBnd(i,2), n(i));
%
% REFERENCE:
% 
% Based on paper: "Multidimensional Triangulation and Interpolation for
% Reinforcement Learning" by Scott Davies, NIPS 1996

% Coerce any data that is out of range:
checkLow = x<xBnd(:,1); x(checkLow) = xBnd(checkLow,1);
checkUpp = x>xBnd(:,2); x(checkUpp) = xBnd(checkUpp,2);

% Translate and scale such that each grid cell is a unit cube:
x = x-xBnd(:,1);
x = x./(xBnd(:,2)- xBnd(:,1));
x = x.*(N-1);

% Seperate out into bin number and fractional length:
xBaseIdx = floor(x);  %Figure out the lower index (bin) number
checkCeil = xBaseIdx==(N-1); newCeil = N-2;
xBaseIdx(checkCeil) = newCeil(checkCeil);  %Fix special case for points on upper grid
x = x-xBaseIdx; %Keep the fraction along each dimension:

% Figure out which dimensions to build simplex along:
[~,I] = sort(x); %x must be a column vector!

% Build simplex by walking along each successive dimension
% The matrix X stores the verticies of the simplex, where each vertex is on
% the unit hypercube. The first vertex is always zeros(n,1) and the last is
% always ones(n,1). The intermediate verticies are selected based on the
% sort step. Since X will be triangular, and all entries are unity, it is
% trivial to solve the system: x = X*w for the unknown weights w by row
% reduction.
n = length(N);
X = zeros(n,n+1);
w = zeros(n+1,1);
wSum = 0;
for i=1:n
    X(I(i), (n+2-i):(n+1)) = 1;
    w(n+2-i) = x(I(i)) - wSum;
    wSum = wSum + w(n+2-i);
end
w(1) = 1-wSum;  %Final convex combination

% Compute the linear indices:
idx = zeros(n+1,1);
for i=1:(n+1)
    idx(i) = sub2idx(N, X(:,i)+xBaseIdx+1, n);
end

end


%%%% Custom version of sub2ind, that takes a vector input for the desired
%%%% input dimension, rather than a comma seperated list.
function idx = sub2idx(n,idxVec,d)
%
% Based on Matlab's sub2ind command
% Modified by Matthew Kelly, April 8, 2015
%
% CHANGES:
%   - must explicitly enter all dimensions: length(n) == length(idxVec)
%   - pass through for scalar: length(n)==1 is valid input
%   - must pass desired subscripts as a vector (not a comma-seperated-list)
%   - disabled most error checking
%
% INPUTS:
%   n = size(dataMatrix)
%   idxVec = list of desired indices. For example A(2,1) -> [2,1]
%
% OUTPUTS:
%   idx = linear index, such that:
%
%       A(idx) = A(idxVec(1), ... , idxVec(length(n));
%

if d == 1
    idx = idxVec;
elseif d ==2
    idx = idxVec(1) + (idxVec(2) - 1).*n(1);
else
    %Compute linear indices
    k = [1 cumprod(n(1:end-1))'];
    idx = 1;
    for i = 1:d
        v = idxVec(i);
        idx = idx + (v-1)*k(i);
    end
end

end
