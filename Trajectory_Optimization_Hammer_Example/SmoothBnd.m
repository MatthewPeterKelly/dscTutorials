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

if nargin==2
    Low = 0;
    Upp = 1;
else
    Low = Bnd(1);
    Upp = Bnd(2);
end

%Apply bounding to each part of the input:
xLow = Low + alpha*log(exp((x-Low)/alpha)+1);
xUpp = Upp - alpha*log(exp(-(x-Upp)/alpha)+1);

%Combine:
xBnd = xLow + xUpp - x;

end

  