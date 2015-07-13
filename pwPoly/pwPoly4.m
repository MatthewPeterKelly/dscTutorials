function x = pwPoly4(tGrid,xGrid,dxGrid,t)
% x = pwPoly4(tGrid,xGrid,t)
%
% This function does piece-wise quadratic interpolation of a set of data.
%
% INPUTS:
%   tGrid = [1, 2*n-1] = time at each grid point
%   xGrid = [m, 2*n-1] = function value at each grid point
%   dxGrid = [m, 2*n-1] = function slope at each grid point
%   t = [1, k] = vector of query times (must be contained within tGrid)
%
% OUTPUTS:
%   x = [m, k] = function value at each query time
%
% NOTES: 
%   If t is out of bounds, then all corresponding values for x are replaced
%   with NaN
%

nGrid = length(tGrid);
if mod(nGrid-1,2)~=0 || nGrid < 3
    error('The number of grid-points must be odd and at least 3');
end

% Figure out sizes
n = floor((length(tGrid)-1)/2);
m = size(xGrid,1);
k = length(t);
x = zeros(m, k);

% Figure out which segment each value of t should be on
edges = [-inf, tGrid(1:2:end), inf];
[~, bin] = histc(t,edges);

% Loop over each quartic segment
for i=1:n
    idx = bin==(i+1);
    if sum(idx) > 0
        gridIdx = 2*(i-1) + [1,2,3];
        x(:,idx) = quartInterp(...
            tGrid(gridIdx), ...
            xGrid(:,gridIdx), ...
            dxGrid(:,gridIdx), ...
            t(idx));
    end
end

% Replace any out-of-bounds queries with NaN
outOfBounds = bin==1 | bin==(n+2);
x(:,outOfBounds) = nan;

end


function x = quartInterp(tGrid,xGrid,dxGrid,t)
%
% This function computes the interpolant over a single interval
%
% INPUTS:
%   tGrid = [1, 3] = time at endpoints and midpoint
%   xGrid = [m, 3] = function at endpoints and midpoint
%   dxGrid = [m, 3] = derivative at endpoints and midpoint
%   t = [1, p] = query times, spanned by tGrid
%
% OUTPUTS:
%   x = [m, p] = function at query times
%

% Rescale the query points to be on the domain [-1,1]
t = 2*(t-tGrid(1))/(tGrid(3)-tGrid(1)) - 1; 

% Unpack function and derivative:
xLow = xGrid(:,1);
xMid = xGrid(:,2);
xUpp = xGrid(:,3);
dxLow = dxGrid(:,1);
dxUpp = dxGrid(:,3);

% Compute the coefficients:
a = dxUpp/4 - dxLow/4 - xLow/2 + xMid - xUpp/2;
b = dxLow/4 + dxUpp/4 + xLow/4 - xUpp/4;
c = dxLow/4 - dxUpp/4 + xLow - 2*xMid + xUpp;
d = (3*xUpp)/4 - dxUpp/4 - (3*xLow)/4 - dxLow/4;
e = xMid;

% Evaluate the polynomial for each dimension of the function:
p = length(t);
m = size(xGrid,1);
x = zeros(m,p);
for i=1:m
    x(i,:) = e(i,:) + t.*(d(i,:) + t.*(c(i,:) + t.*(b(i,:) + t.*a(i,:))));
end

end


