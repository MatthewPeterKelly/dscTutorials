function [p,w, fVal, exitFlag] = fitBezierCurve(t,x,nPoint,tSpan, xBnd)
% [p,w, fVal, exitFlag] = fitBezierCurve(t,x,nPoint,tSpan, xBnd)
%
% This function fits a rational bezier curve to the data x(t), using a
% bezier curve with n points.
%
% INPUTS:
%   t = [1 x nTime] monotonically increasing time vector
%   x = [nCurve x nTime] vector function at each point in t
%   nPoint = number of points to use for bezier curve
%   tSpan = [1 x 2] domain of t
%   xBnd = [nCurve x 2] = [lower, upper] bound on function
%
% OUTPUTS:
%   p = [nCurve x nPoint] = bezier control points
%   w = [1 x nPoint] = weights for each control point
%   fVal = mean-square-error in function fit
%   exitFlag = fmincon exit flag
%
% See also: bezierCurve, rationalBezierCurve
%

nCurve = size(x,1);

tNormalized = (t-tSpan(1))/diff(tSpan);
wGuess = 0.5*ones(1,nPoint);
tGuess = linspace(0,1,nPoint);
pGuess = interp1(tNormalized',x',tGuess,'linear','extrap')';
zGuess = packDecVar(pGuess,wGuess);

wLow = zeros(1,nPoint);
wUpp = ones(1,nPoint);
pLow = xBnd(:,1)*ones(1,nPoint);
pUpp = xBnd(:,2)*ones(1,nPoint);

zLow = packDecVar(pLow,wLow);
zUpp = packDecVar(pUpp,wUpp);

options = optimset(...
    'Display','off');

problem.Aineq = []; problem.Aeq = [];
problem.bineq = []; problem.beq = [];
problem.lb = zLow;
problem.ub = zUpp;
problem.x0 = zGuess;
problem.options = options;
problem.objective = @(z)evalFit(z,t,x,nCurve,nPoint, tSpan);
problem.solver = 'fmincon';

[zSoln,fVal,exitFlag] = fmincon(problem);

[p,w] = unpackDecVar(zSoln,nCurve,nPoint);

end



function [p,w] = unpackDecVar(z,nCurve,nPoint)

w = z(1:nPoint)';

p = reshape(z(nPoint+1:end),nCurve,nPoint);

end


function z = packDecVar(p,w)

[nCurve, nPoint] = size(p);

z = [w'; reshape(p,nCurve*nPoint,1)];

end


function mse = evalFit(z,t,x,nCurve,nPoint,tSpan)

[p,w] = unpackDecVar(z,nCurve,nPoint);
xFit = rationalBezierCurve(p,w,t,tSpan);

mse = mean(mean((xFit-x).^2));

end

