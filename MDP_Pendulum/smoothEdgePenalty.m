function y = smoothEdgePenalty(x,xBnd,xDel,yMax)
% y = smoothEdgePenalty(x,xBnd,xDel,yMax)
%
% This function computes a penalty curve, that is near yMax at each point
% in xBnd, and near zero at mean(xBnd). The paramter xDel adjusts how
% rapidly the function changes from ~0 -> ~1. It is based on the hyperbolic
% tangent function.
%
% INPUTS:
%   x = list of queries
%   xBnd = range of x
%   xDel = width of boundry
%   yMax = function height
%

m = 3/xDel;
yUpp = tanh(m*(x-xBnd(2)));
yLow = tanh(-m*(x-xBnd(1)));
y = yMax*(2 + yLow + yUpp);

end