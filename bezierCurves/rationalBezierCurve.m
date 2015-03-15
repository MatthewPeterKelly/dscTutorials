function x = rationalBezierCurve(p,w,t,tSpan)
% x = rationalBezierCurve(p,w,t,tSpan)
%
% This function evaluates a rational bezier curve, defined by points p and
% weights w.
%
% INPUTS:
%   p = [nCurve x nPoint] = control points
%   w = [1 x nPoint] = control point weights
%   t = [1 x nTime] = times at which to evaluate curve
%   tSpan = [1 x 2] = tSpan(1) <= t <= tSpan(2)
%
% OUTPUTS:
%   x = [nCurve x nTime] = bezier curve, evaluated at t
%
% NOTES:
%   It is not advisable to use this function for high-order polynomials.
%
% See also: BEZIERCURVE, FITBEZIERCURVE
%

[nCurve, nPoint] = size(p);
nTime = length(t);

t = (t-tSpan(1))/diff(tSpan);

%%% Compute the numerator and denominator:
num = zeros(nCurve,nTime);
den = zeros(nCurve,nTime);
ONE = ones(nCurve,1);
n = nPoint - 1;
for i=0:n
    tt = (t.^i).*(1-t).^(n-i);
    binom = nchoosek(n,i);
    num = num + binom *w(i+1)*p(:,i+1)*tt;
    den = den + binom *w(i+1)*ONE*tt;
end
x = num./den;

end