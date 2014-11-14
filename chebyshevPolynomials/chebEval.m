function y = chebEval(f,t,d)
% function y = chebEval(f,t,d)
%
% This is a stripped-down version of chebyshevInterpolate, and is designed
% to only be called as a mex function.
%
% This function uses barycentric interpolation to evaluate a chebyshev
% polynomial. Technical details are taken from the book: Approximation
% Theory and Approximation Practice by Lloyd Trefethen. The core equation
% that I use can be found as Theorem 5.2 in this book, and is called
% Barycentric Interpolation.
%
% Written by Matthew Kelly
% October 27, 2013
%   Updated: November 8, 2013
%   Updated: October 17, 2014
% Cornell University
%
% This function computes the value of the chebyshev polynomial that is
% defined by the vector f, at the inputs t. It can also be used to return
% the first and second derivatives of y with respect to t.
%
% INPUTS:
%   f = [KxN] value of the chebyshev polynomial at each of the chebyshev
%       points (these can be computed using chebyshevPoints.m). Each row
%       represents a single set of chebyshev node values for a single
%       state.
%   t = [1xM] vector of inputs to be evaluated (monotonically increasing)
%   d = [1x2] vector speficying the domain of the polynomial. d(2) > d(1)
%
% OUTPUTS:
%   y = [KxM] the value of the interpolant at each point in t
%
% NOTES:
%   Any points in t that are not in the domain d will be evaluated at
%   either d(1) or d(2), whichever is closer. This avoids strange end
%   effects when the function is evaluated outside of its domain.
%
% What is happening here, in plain english:
%
%   There are several Chebyshev points (or nodes) that are spread out
%   across the interval d using a special grid spacing. We will call these
%   points x. For each of the points in x, there is a corresponding value
%   of the chebyshev function, we'll call it f.
%
%   Let's say that we want to find the value of the approximation for some
%   input that is not in x. This can be done using a interpolation of the
%   values in f:
%                   y = c(t,x).*f
%
%   Note that the weighting terms are a function of both the desired input
%   and the chebyshev points. It turns out that there is a more stable and
%   efficient way to calculate this interpolation, which is roughly of the
%   form:
%                   y = (k(t,x).*f)/sum(k(t,x))
%
%   This code evaluates k(t,x) which it then uses to evaluate and return y.
%
%
% NOTE - The base algorithm (described above) is not defined for points in
% t that are close to the chebyshev nodes in x. If a point in t matches a
% point in x, then an additional claculation is required. If a point in t
% is very close to a gridpoint, then there is a slight loss of accuracy,
% which is more pronounced.
%

[k,n] = size(f);
m = length(t);
x = chebPtsFast(n,d);   %Get the chebyshev points:

y = zeros(k,m);

%Loop over each point in t:
for i=1:m
    if t(i)<d(1), t(i) = d(1);  %Input Checking and clamping
    elseif t(i)>d(2), t(i) = d(2); end;
    num = zeros(k,1); den = 0;
    for j=1:n  %Loop over each chebyshev node:
        dist = t(i) - x(j);
        if abs(dist) < 10*eps
            y(:,i) = f(:,j);  %Then we're sitting on a node
            break;
        else
            val = 1/dist;
            if mod(j,2)==1, val=-val; end;
            if j==1 || j==n
                num = num + 0.5*f(:,j)*val;
                den = den + 0.5*val;
            else
                num = num + f(:,j)*val;
                den = den + val;
            end
            y(:,i) = num/den;
        end
    end
end

end

function x = chebPtsFast(n,d)
%
% This function is a light-weight version of the function: chebpts.m
% written by Lloyd Trefethen as part of his Chebyshev Polynomial matlab
% toolbox: chebyfun, which can be downloaded from:
% http://www2.maths.ox.ac.uk/chebfun/download/
%
% The algorithm for computing the quadrature weights is also from
% trefethen's toolbox, with the citation:
% Jörg Waldvogel, "Fast construction of the Fejér and Clenshaw-Curtis
% quadrature rules", BIT Numerical Mathematics 43 (1), p. 001-018 (2004).
% http://www2.maths.ox.ac.uk/chebfun/and_beyond/programme/slides/wald.pdf
%
% Slight modifications made by Matthew Kelly
% October 27, 2013
% Cornell University
%
% This function returns the n chebyshev points, over the interval d. Error
% checking has not been included on the inputs.
%
% INPUTS:
%   n = [1x1] the desired number of chebyshev points
%   d = [1x2] domain of the polynomial. Default = [-1,1]
%
% OUTPUTS: (2nd-kind chebyshev points and weights)
%   x = [1xn] the n chebyshev points over the interval d
%   w = [1xn] the n chebyshev weights for Clenshaw-Curtis quadrature
%


if n == 1, x = 0; return, end % Special case

%Compute the chebyshev points on the domain [-1,1]:
m = n-1;
x = sin(pi*(-m:2:m)/(2*m));       % Chebyshev points

%Rescale (if necessary):
if nargin~=1
    x = (diff(d)*x + sum(d))/2;
end

end
