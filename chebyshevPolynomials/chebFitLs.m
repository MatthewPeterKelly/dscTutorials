function [f,d,x] = chebFitLs(t,y,d,n)
%[f,d,x] = chebFitLs(t,y,d,n)
%
% FUNCTION:
%   This function fits chebyshev polynomial of order n to data using a lest
%   squares fit. It is much slower than chebyshevFit, but is more accurate
%   for data that is noisy. This can be slow if the order if length(t) or n
%   is too large. 
%
% INPUTS:
%   t = data input
%   y = data output
%   d = domain
%   n = order of the fit (non-negative integer)
%
% OUTPUTS:
%   f = [1x(n+1)] vector of values of the curve fit at each chebyshev node
%   d = [1x2] vector giving the domain of the curve fit
%   x = [1x(n+1)] vector of the chebyshev points used for the fit
%
%
% NOTES:
%   d must satisfy:
%       d(1) <= min(t) < max(t) <= d(2)
%
%   n must satisfy:
%       n < length(t)
%

x = chebyshevPoints(n+1,d);
fGuess = interp1(t',y',x','pchip','extrap')';

if exist('chebEval_mex','file')
    func = @(f)getFitError_mex(f,t,d,y);
else
    func = @(f)getFitError(f,t,d,y);
    disp('Warning: could not find chebEval_mex, using chebEval instead. This will make computation very slow.');
end

%Now use a more sophisticated method to refine f
f = fminsearch(func,fGuess);

end

function mse = getFitError(fGuess,x,d,y)

yGuess = chebEval(fGuess,x,d);
mse = mean((y-yGuess).^2);

end

function mse = getFitError_mex(fGuess,x,d,y)

yGuess = chebEval_mex(fGuess,x,d);
mse = mean((y-yGuess).^2);

end


