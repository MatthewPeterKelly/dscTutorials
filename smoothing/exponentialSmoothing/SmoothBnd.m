function xBnd = SmoothBnd(x,alpha,Bnd)
%
% xBnd = SmoothBnd(x,alpha,Bnd)
%
% This function is used to smoothly bound the input x;
% 
% INPUTS:
%   x = a vector of real numbers to apply bounds
%   alpha = a positive smoothing parameter. Small values correspond to
%           little smoothing
%   Bnd = the desired bounds of the function, expressed as a 2-element row
%         vector. If ommitted it will default to [0,1];
%
% Written by Matthew Kelly
% October 2013
% Cornell University
%

if nargin==2
    Low = 0;
    Upp = 1;
else
    Low = Bnd(1);
    Upp = Bnd(2);
end

infTest1 = exp(max(x-Low)/alpha);
infTest2 = exp(max(-(x-Upp))/alpha);

if isinf(infTest1) || isinf(infTest2)  %Then there is a sharp transition
    xBnd = x;
    xBnd(x<Low) = Low;
    xBnd(x>Upp) = Upp;
else
    %Apply bounding to each part of the input:
    xLow = Low + alpha*log(exp((x-Low)/alpha)+1);
    xUpp = Upp - alpha*log(exp(-(x-Upp)/alpha)+1);
    
    %Combine:
    xBnd = xLow + xUpp - x;
end

end

  