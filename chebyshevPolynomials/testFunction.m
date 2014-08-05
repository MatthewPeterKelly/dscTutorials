function [y, Dy, DDy, DDDy] = testFunction(x)
%
% FUNCTION:
%   This function is used to test various aspects of the chebyshev
%   interpolants. For now the test function is a sine wave multiplied by an
%   exponential function.
%
% INPUTS:
%   x = [1 x N] input vector
%
% OUTPUTS:
%   y = [1 x N] output vector
%   Dy = First derivative of the output with respect to the input
%   DDy = Second derivative
%   DDDy = Third derivative
%
% NOTES:
%   The derivatives are computed using the matlab symbolic toolbox. The
%   code to do this is included at the end of the function.
%

%Parameters for the function
a = 2.0;    %amplitude
b = 17;     %frequency
c = 0.7;    %phase
d = 0.8;    %time constant

%compute the test function
y = a*sin(b*x+c).*(1-exp(d*x));

%compute higher order derivatives if desired
if nargout >= 2
    Dy = - a*d*exp(d*x).*sin(c + b*x) - a*b*cos(c + b*x).*(exp(d*x) - 1);
    if nargout >= 3
        DDy = a*b^2*sin(c + b*x).*(exp(d*x) - 1) - a*d^2*exp(d*x).*sin(c + b*x) - 2*a*b*d*cos(c + b*x).*exp(d*x);
        if nargout >= 4
            DDDy = a*b^3*cos(c + b*x).*(exp(d*x) - 1) - a*d^3*exp(d*x).*sin(c + b*x) - 3*a*b*d^2*cos(c + b*x).*exp(d*x) + 3*a*b^2*d*exp(d*x).*sin(c + b*x);
        end
    end
    
end


% % % % %%Used to generate the symbolic derivatives:
% % % % syms a b c d x;
% % % % y = a*sin(b*x+c).*(1-exp(d*x));
% % % % Dy = diff(y,x);
% % % % DDy = diff(Dy,x);
% % % % DDDy = diff(DDy,x);