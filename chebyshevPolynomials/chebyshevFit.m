function [f,d,x] = chebyshevFit(IO, n)
%[f,d,x] = chebyshevFit(IO, n)
%
% FUNCTION:
%   This function fits chebyshev polynomial of order n to the data or
%   function handle in the struct IO.
%
% INPUTS:
%   Call_Sequence_data:
%       IO.input = [1xN] vector of input data
%       IO.output = [1xN] vector of output data
%   Call_Sequence_function:
%       IO.userFunc = [function handle]
%           y = f(t);
%               y = a scalar output
%               t = a scalar input
%       IO.domain = [1x2] vector giving the domain of the curve fit:
%                   [lower, upper]
%   n = order of the fit (non-negative integer)
%
% OUTPUTS:
%   f = [1x(n+1)] vector of values of the curve fit at each chebyshev node
%   d = [1x2] vector giving the domain of the curve fit
%   x = [1x(n+1)] vector of the chebyshev points used for the fit
%

if isfield(IO,'input')
    %get value of the function at the chebyshev nodes via interpolation:
    d = [min(IO.input), max(IO.input)];
    x = chebyshevPoints(n+1,d);
    f = interp1(IO.input',IO.output',x','pchip')';
    
elseif isfield(IO,'userFunc')
    %Then we are working from an analytic function
    d = IO.domain;
    x = chebyshevPoints(n+1,d);
    f = feval(IO.userFunc,x);
    
else
    error('Invalid fields in argument: IO')
    
end

end

function mse = getFitError(fGuess,x,d,y)

yGuess = chebEval(fGuess,x,d);
mse = mean((y-yGuess).^2);

end
