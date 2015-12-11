function xMax = SmoothMax(x,alpha)
%
% xMax = SmoothMax(x, alpha)
%
% This function is a smooth version of the max(x) function, commonly known
% as the 'soft-max' function. In the limit as the smoothing parameter goes
% to zero, this function will return max(x). In the limit as the smoothing
% parameter goes to infinity, this will return sum(x).
%
%INPUTS:
% x = a vector or matrix of inputs 
% alpha = a smoothing parameter. Asymtotes:  0->max(x), inf->sum(x) 
%
%OUTPUTS:
% xMax = the smooth maximum of x
%
%
% Written by Matthew Kelly
% October 2013
% Cornell University
%

if nargin==1
    alpha = 1;
end

xMax = alpha*log(sum(exp(x/alpha)));

if isinf(xMax)
   %Then the scaling was too sharp
    xMax = max(x);
end

end