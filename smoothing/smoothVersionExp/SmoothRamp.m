function y = SmoothRamp(x,alpha)

%y = SmoothRamp(x,alpha)
%
% This function is used to make a smooth version of the ramp function
%
%INPUTS:
% x = a vector of real numbers to be smoothed
% alpha = a positive smoothing parameter. Small values of alpha correspond
%         to little smoothing, large values correspond to heavy smoothing.
%         If alpha is too small, no smoothing will be done.
%
% Written by Matthew Kelly
% October 2013
% Cornell University
%

%Check for infinities
infTest = exp(max(x)/alpha);

if isinf(infTest)   %Then there is a sharp transition
    y = x;
    y(x<0) = 0;
else  %do smoothing
    y = alpha*log(exp(x/alpha)+1);
end

end