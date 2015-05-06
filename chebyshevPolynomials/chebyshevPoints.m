function [x, w] = chebyshevPoints(n,d)
%[x, w] = chebyshevPoints(n,d)
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

%Check if weights are needed:
if nargout==2
    w = weights(n)*diff(d)/2;
end
end

function w = weights(n) % 2nd-kind Chebyshev wieghts
% Jörg Waldvogel, "Fast construction of the Fejér and Clenshaw-Curtis 
% quadrature rules", BIT Numerical Mathematics 43 (1), p. 001-018 (2004).
% http://www2.maths.ox.ac.uk/chebfun/and_beyond/programme/slides/wald.pdf
if n == 1
    w = 2;
else
    % new
    n = n-1;
    u0 = 1/(n^2-1+mod(n,2));                      % Boundary weights
    L = 0:n-1; r = 2./(1-4*min(L,n-L).^2);        % Auxiliary vectors
    w = [ifft(r-u0) u0];                          % C-C weights
end
end