function lxl = SmoothAbsFancy(x,alpha,mLow,mUpp)
%
%lXl = SmoothAbs(x,alpha)
%lxl = SmoothAbs(x,alpha,mLow,mUpp)
%
%This function is a smooth version of absolute value. There are two
%versions of the function. If called with two arguments it will use
%hyperbolic tangent smoothing, and if called with 4 arguments it will use a
%fancy smoothing that allows for the slope to be adjusted on each side.
%
%INPUTS:
% x = a vector of inputs to smooth
% alpha = smoothing parameter, alpha > 0.
%           Small alpha (1e-4) => no smoothing
%           large alpha (1e-1) => heavy smoothing
%           (alpha sample values assumes x on the order of 1)
% mLow = the slope of the I/O asymtote for positive x - default 1
% mUpp = the slope of the I/O asymtote for negative x - default 1
%
%OUTPUTS:
% lxl = the smooth version of x
%
%
% Written by Matthew Kelly
% October 2013
% Cornell University
%

infTest = exp(max(x)/alpha);
if isinf(infTest)
    lxl = mUpp*x;
    lxl(x<0) = -mLow*x(x<0);
else
    lxl = alpha*(...
        mLow*log(exp(-x/alpha)+1)+...   %x>0 part
        mUpp*log(exp(x/alpha)+1)...     %x<0 part
        );
end


end