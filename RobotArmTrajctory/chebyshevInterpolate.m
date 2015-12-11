function [y, Dy, DDy, DDDy] = chebyshevInterpolate(f,t,d)
%[y, Dy, DDy, DDDy] = chebyshevInterpolate(f,t,d)
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
%   d = [1x2] vector speficying the domain of the polynomial
%
% OUTPUTS:
%   y = [KxM] the value of the interpolant at each point in t
%   Dy = first derivative of y with respect to t
%   DDy = second derivative of y with respect to t
%   DDDy = third derivative of y with respect to t
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

%Check to see if the t vector is on the proper domain:
if min(t) < d(1) || max(t) > d(2)
   disp('WARNING - the input to chebyshevInterpolate does not lie on the domain that was specified for the function')
   disp('    --> This can lead to unpredictable outputs and is not suggested.')   
end

%Get the chebyshev points
[k,n] = size(f);
x = chebyshevPoints(n,d);
ONE1 = ones(k,1);
ONE2 = ones(1,length(t));

%Loop through each chebyshev node.
num = zeros(k,length(t));
den = zeros(k,length(t));
for i=1:n
    val = ONE1*(1./(t-x(i)));
    if mod(i,2)==1, val=-val; end;
    if i==1 || i==n
        num = num + 0.5*(f(:,i)*ONE2).*val;
        den = den + 0.5*(val);
    else
        num = num + (f(:,i)*ONE2).*val;
        den = den + val;
    end
end

%compute the solution:
y = num./den;

%Check for any values that were too close to nodes and correct them
nanIdx = isnan(y);
if sum(sum(nanIdx))>0 
    nanRowIdx = max(nanIdx,[],1);
    y(:,nanRowIdx) = interp1(x',f',t(nanRowIdx)')';
end


%%%% Derivative Calculations %%%%
if nargout == 2
    Df = chebyshevDerivative(f,d);
    Dy = chebyshevInterpolate(Df,t,d); 
elseif nargout == 3
    [Df, DDf] = chebyshevDerivative(f,d);
    Dy = chebyshevInterpolate(Df,t,d); 
    DDy = chebyshevInterpolate(DDf,t,d); 
elseif nargout == 4
    [Df, DDf, DDDf] = chebyshevDerivative(f,d);
    Dy = chebyshevInterpolate(Df,t,d); 
    DDy = chebyshevInterpolate(DDf,t,d); 
    DDDy = chebyshevInterpolate(DDDf,t,d); 
end

    


end


 
% %  % This block of code provides a more efficient way of computing the
% %  % derivative of the function for very high order polynomial fits. 
% %  %
% %  % It turns out that computing the differentiation matrix, which is done in
% %  % chebyshevDerivative, is an order n^2 calculation, which takes a long
% %  % time to compute for high order fits. This code is faster, but it is
% %  % not as accurate, especially near (or on) the gridpoints.
% %  %
% %  % To implement this code, put it between the lines:
% %  %    --> %%%% Derivative Calculations %%%%
% %  %    --> if nargout == 2
% %  %          CODE HERE
% %  %    --> end
% %  % Note that this would prevent the code from calculating higher order
% %  % derivatives.
% % 
% %     
% %
% %     %Loop through each chebyshev node again, slightly different math
% %     numDer = zeros(1,length(t));
% %     denDer = zeros(1,length(t));
% %     for i=1:n
% %         val = 1./(t-x(i)).^2;
% %         if mod(i,2)==1, val=-val; end;
% %         if i==1 || i==n
% %             numDer = numDer - 0.5*f(i).*(val);
% %             denDer = denDer - 0.5*(val);
% %         else
% %             numDer = numDer - f(i).*val;
% %             denDer = denDer - val;
% %         end
% %     end
% %     
% %     %Compute the derivative of the approximation
% %     Dy = (numDer.*den - num.*denDer)./(den.^2);
% %     
% %     %Check for any values that were to close to nodes
% %     %    -> NOTE - It is expensive to call chebyshevDerivative because it 
% %     %       it computes a huge matrix just to take one or two derivatives.
% %     %    -> If speed is more important than accuracy, replace the contents
% %     %       of the folloging if statement with the code in comments after
% %     %       the end of the function. This uses a 4th order accurate central
% %     %       difference to compute the derivative.
% %     %
% %     nanIdx = isnan(Dy);
% %     nanSum = sum(nanIdx);
% %     if nanSum>0
% % 
% %  %Use a 4th order central difference to get any values that were Nan
% %         
% %         %This was determined experimentally:
% %         %  --> Too Small - run into machine precision
% %         %  --> Too Big - doesn't capture local shape well
% %         SMALL_NUMBER = 10^(-2*log10(length(f)));
% %         
% %         %Get a vector of times that we need function values at
% %         timeDiff = [...
% %             t(nanIdx) - 2*SMALL_NUMBER;
% %             t(nanIdx) - SMALL_NUMBER;
% %             t(nanIdx) + SMALL_NUMBER;
% %             t(nanIdx) + 2*SMALL_NUMBER];
% %         timeDiffVec = reshape(timeDiff,4*nanSum,1)';
% %         
% %         %Get the values of the function at each point in timeDiff
% %         funcDiffVec = chebyshevInterpolate(f, timeDiffVec,d)';
% %         funcDiff = reshape(funcDiffVec,4,nanSum);
% %         
% %         %From Wikipedia: Finite difference coefficient
% %         %http://en.wikipedia.org/wiki/Finite_difference_coefficient
% %         coeff = [1/12, -2/3, 2/3, -1/12];
% %         
% %         %Correct the Nan values
% %         Dy(nanIdx) = sum(coeff*funcDiff,1)/SMALL_NUMBER;
% %     end

