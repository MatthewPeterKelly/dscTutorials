function [Df, DDf, DDDf] = chebyshevDerivative(f,d)
%[Df, DDf, DDDf] = chebyshevDerivative(f,d)
%
% FUNCTION:
%   This function computes the derivative(s) of the chebyshev interpolant
%   at each of the chebyshev nodes.
%
% INPUTS:
%   f = [nState x nPoints] values at each chebyshev node
%   d = [1 x 2] domain of the chebyshev function
%
% OUTPUTS:
%   Df = the derivative of the chebyshev interpolant at each chebyshev node
%   DDf = the second derivative at each chebyshev node
%   DDDf = the third derivative at each chebyshev node
%
%
% NOTES:
%   The derivative at each node is computed by multiplying f by a
%   differentiation matrix. This matrix is [nPoints x nPoints]. If f is a
%   very large order interpolant then computing this matrix may max out the
%   memory available to matlab.
%
%   It is fairly expensive to compute the differentiation matrix, but it is
%   constant for a given order and domain chebyshev approximation. To take
%   advantage of this, the differentiation matrix is stored as a persistent
%   variable.
%


%It is expensive to compute D, so don't calculate it again if the length of
%the input has not changed.
persistent D DD DDD nPrev nOutPrev dPrev

n = size(f,2);

%Check if the input has changed:
ReCalculate = true;
if ~isempty(nPrev)  %Then the function has been called before...
    Conditions = [...
        nPrev == n,...          %Same number of grid points
        nOutPrev == nargout,... %Same number of outputs
        dPrev == d];            %Same domain
    ReCalculate = ~(sum(Conditions)==4);
end

if ReCalculate;  %Then at least one condition fails, so we need to
  %Compute D, DD, and DDD

    D = chebyshevDifferentiationMatrix(n,d);

    %Save persistent variables:
    nPrev = n;
    nOutPrev = nargout;
    dPrev = d;
    
    %Compute and store higher-order differentiation matricies
    if nargout==1
        DD = [];
        DDD = [];
    elseif nargout==2
        DD=D*D;
        DDD = [];
    elseif nargout==3
        DD=D*D;
        DDD=D*DD;
    end
end

%Apply the differentiation matrix
    if nargout==1
        Df = (D*f')';
    elseif nargout==2
        Df = (D*f')';
        DDf = (DD*f')';
    elseif nargout==3
        Df = (D*f')';
        DDf = (DD*f')';
        DDDf = (DDD*f')';
    end

end
