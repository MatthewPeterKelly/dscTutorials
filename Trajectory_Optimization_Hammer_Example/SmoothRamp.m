function y = SmoothRamp(x,alpha)

%y = SmoothRamp(x,alpha)
%
% This function is used to make a smooth version of the ramp function
%
%INPUTS:
% x = a vector of real numbers to be smoothed
% alpha = a positive smoothing parameter. Small values of alpha correspond
%         to little smoothing, large values correspond to heavy smoothing

y = alpha*log(exp(x/alpha)+1);

end