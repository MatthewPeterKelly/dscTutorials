function [f,d,x,mse] = chebFitLs(t,y,d,n)
%[f,d,x] = chebFitLs(t,y,d,n)
%
% FUNCTION:
%   This function fits chebyshev polynomial of order n to data using a lest
%   squares fit. It is much slower than chebyshevFit, but is more accurate
%   for data that is noisy. This can be slow if the order if length(t) or n
%   is too large.
%
% INPUTS:
%   t = [1 x nTime] data input
%   y = [nState x nTime] data output
%   d = [1 x 2] domain
%   n = order of the fit (non-negative integer)
%
% OUTPUTS:
%   f = [nState x (n+1)] vector of values of the curve fit at each chebyshev node
%   d = [1x2] vector giving the domain of the curve fit
%   x = [1 x (n+1)] vector of the chebyshev points used for the fit
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
fGuess = interp1(t',y',x','spline','extrap')';

%Now use a more sophisticated method to refine f
nState = size(fGuess,1);
f = zeros(size(fGuess));
mse = zeros(nState,1);
for i=1:nState
    if exist('chebEval_mex','file')
        func = @(f)getFitError_mex(f,t,d,y(i,:));
    else
        func = @(f)getFitError(f,t,d,y(i,:));
        disp('Warning: could not find chebEval_mex, using chebEval instead. This will make computation very slow.');
    end
    [f(i,:), mse(i)] = fminsearch(func,fGuess(i,:));
end

end

function mse = getFitError(fGuess,x,d,y)

yGuess = chebEval(fGuess,x,d);
mse = mean((y-yGuess).^2);

end

function mse = getFitError_mex(fGuess,x,d,y)

yGuess = chebEval_mex(fGuess,x,d);
mse = mean((y-yGuess).^2);

end


