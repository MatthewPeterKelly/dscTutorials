function If = chebyshevIntegral(f,d)
%If = chebyshevIntegral(f,d)
%
% FUNCTION:
%   This function computes the integrals of the chebyshev approximation
%   using Clenshaw-Curtis quadrature.
%
% INPUTS:
%   f = [nState x nPoints] values at each chebyshev node
%   d = [1 x 2] domain of the chebyshev function
%
% OUTPUTS:
%   If = the integral of the chebyshev interpolant at each chebyshev node,
%   with the integral defined to be zero at the first node.
%

%Get the quadrature weights:
[k,n] = size(f);
[~,w] = chebyshevPoints(n,d);

%Perform the integral
ONE = ones(k,1);  %get the matrix size correct for .* operation
If = cumsum(f.*(ONE*w));

end
