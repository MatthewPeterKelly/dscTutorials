function [pp, mse] = fitSpline(tData, xData, tKnot)
% pp = fitSpline(tData, xData, tGrid)
%
% Fits a piece-wise cubic spline to {tData, xData} using the
% knot-points in tKnot.
%
% INPUTS:
%     tData = [1,nData] = time stamps for data (uniform spacing, monotonic!)
%     xData = [nFunc,nData] = function value at each time
%     tKnot = [1,nKnot] = time for each knot point (monotonic!)
%  
%   OUTPUTS:
%     pp = struct to be evaluated by Matlab's ppval command
%  
%   NOTES:   (strongly suggested)
%   
%     tData([1,end]) == tKnot([1,end])    
%

nFunc = size(xData,1);
nKnot = length(tKnot);

xGuess = interp1(tData',xData',tKnot')';

%Numerically estimate the local slope
vData = zeros(size(xData));
for i=1:nFunc
vData(i,:) = diffCenter(xData(i,:),mean(diff(tData)));
end

vGuess = interp1(tData',vData',tKnot')';
zGuess = reshape([xGuess;vGuess],2*nFunc*nKnot,1);
options = optimoptions('fminunc',...
    'MaxFunEvals',200*length(zGuess),...
    'Display','off',...
    'Algorithm','quasi-newton');

objFun = @(z)checkFit(z,tData,xData,tKnot,nFunc,nKnot);
zSoln = fminunc(objFun,zGuess,options);

% Check the solution and return the answer:
[mse, pp] = checkFit(zSoln,tData,xData,tKnot,nFunc,nKnot);

end



function [mse, pp] = checkFit(z,tData,xData,tKnot,nFunc,nKnot)

zKnot = reshape(z,2*nFunc,nKnot);
xKnot = zKnot(1:nFunc,:);
vKnot = zKnot((nFunc+1):end,:);

pp = pwch(tKnot,xKnot,vKnot);
xFit = ppval(pp,tData);
mse = sum(mean((xFit-xData).^2));

end


function dx = diffCenter(x,dt)
% dx = diffCenter(x,dt)
%
%   Computes the second-order finite difference approximation of x with
%   respect to t. A one-sided second order difference is used at the end
%   points, so size(dx) == size(x).
%
%   INPUTS:
%       x = [1 x n] vector of function values 
%       dt = sampling period of x   (default = 1)
%
%   OUTPUTS:
%       d = dx/dt = first derivative of x wrt t
%
% See also: cumInt, diff

if nargin == 1
    dt = 1;
end

if length(dt) ~= 1
    error('Time-step (dt) must be a scalar');
end

n = length(x);
if n < 2
    error('length(x) must be at least 2');
elseif n == 2
    dx = diff(x)/dt;
else %Then enough points for second-order finite difference
    dx = zeros(size(x));
    Dx = diff(x);   %Matlab first-order finite difference
    dx(1) = (-3*x(1) + 4*x(2) - x(3))/(2*dt);
    dx(2:(end-1)) = (Dx(1:(end-1)) + Dx(2:end))/(2*dt);  %Mid-points are easy
    dx(end) = (x(end-2) - 4*x(end-1) + 3*x(end))/(2*dt);
end

end
