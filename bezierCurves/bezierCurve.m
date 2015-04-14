function x = bezierCurve(p,t,tSpan)
% x = bezierCurve(p,t,tSpan)
%
% This function evaluates a bezier curve, defined by the set of control
% points p, at each value in t.
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
%   It is not advisable to use this function for high-order polynomials.
%
% See also: RATIONALBEZIERCURVE, FITBEZIERCURVE
%

[nCurve, nPoint] = size(p);
nTime = length(t);

t = (t-tSpan(1))/diff(tSpan);

x = zeros(nCurve,nTime);
n = nPoint - 1;
for i=0:n
    tt = (t.^i).*(1-t).^(n-i);
    binom = nchoosek(n,i);
    x = x + binom*p(:,i+1)*tt;
end

end
