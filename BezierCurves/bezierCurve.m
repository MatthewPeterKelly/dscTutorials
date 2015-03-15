function x = bezierCurve(p,t,tSpan)
% x = bezierCurve(p,t,tSpan)
%
% This function evaluates a bezier curve, defined by the set of control
% points p, at each value in t, using a recursive algorithm.
%
% INPUTS:
%   p = [nCurve x nPoint] = control points
%   t = [1 x nTime] = times at which to evaluate curve
%   tSpan = [1 x 2] = tSpan(1) <= t <= tSpan(2)
%
% OUTPUTS:
%   x = [nCurve x nTime] = bezier curve, evaluated at t
%
% NOTES:
%   It is not advisable to use this function for high-order polynomials, as
%   the recursive evaluation will propagate numerical instabilities. For
%   high-order function approximation, it is better to use the barycentric
%   form of the Chebyshev polynomials, or something similar.
%

[nCurve, nPoint] = size(p);

% Rescale time to [0,1]
tt = ones(nCurve,1)*(t-tSpan(1))/diff(tSpan);

% Call the recursive version of the function
x = bezierKernel(p,tt,1,nPoint);

end

%%%% SUB FUNCTIONS: %%%%

function x = bezierKernel(p,t,iLow,iUpp)
%
% INPUTS:
%   p = [nCurve x nPoint] = control points
%   t = [nCurve x nTime] = time;  0 < t < 1
%
% OUTPUTS:
%   x = [nCurve x nTime] = bezier curve, evaluated at t
%

if iLow ==iUpp  %Reached base case - end recursion
    x = p(:,iLow)*ones(1,size(t,2));
else
    xLow = bezierKernel(p,t, iLow, iUpp-1);
    xUpp = bezierKernel(p,t, iLow+1, iUpp);
    x = (1-t).*xLow + t.*xUpp;
end


end