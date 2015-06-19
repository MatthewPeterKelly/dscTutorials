function [x,w] = chebyshevScalePoints(xx,ww,d)
% [x,w] = chebyshevScalePoints(xx,ww,d)
%
% This function scales the chebyshev points to an arbitrary interval
%
% INPUTS:
%   xx = chebyshev points on the domain [-1,1]
%   ww = chebysehv weights on the domain [-1,1]
%   d = [low, upp] = new domain
%
% OUTPUTS:
%   x = chebyshev points on the new domain d
%   w = chebyshev weights on the new domain d
%

    x = ((d(2)-d(1))*xx + sum(d))/2;
    w = ww*(d(2)-d(1))/2;

end