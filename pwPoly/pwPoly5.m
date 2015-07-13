function [x,dx,ddx] = pwPoly5(T,P,t)
% [x,dx,ddx] = pwPoly5(T,P,t)
%
% This function computes a piece-wise 5th-order polynomial that
% interpolates the points in P, which contain function value, slope, and
% curvature information.
%
% INPUTS:
%   T = [1, nKnot] vector of the time at each knot point
%   P = [3, nKnot] = [x;dx;ddx] = function data at each knot point
%   t = [1, nTime] = vector of desired interpolation times
%
% OUTPUTS:
%   x = [1, nTime] = function value
%   dx = [1, nTime] = first time derivative of x
%   ddx = [1, nTime] = second time derivative of x
%
% ASSUME:
%   --> T is monotonicall increasing: diff(T) > 0
%   --> All inputs contain only real numbers
%   --> T(1) <= min(t) <= T(end)
%

% All points are considered valid, so extend edge bins:
Tbins = [-inf, T(2:(end-1)), inf];

%Figure out which bins each query is in:
[~, idx] = histc(t,Tbins);

% Call a different sub-function depending on number of outputs:
if nargout == 1
    x = subFun1(T,P,t,idx);
elseif nargout == 2
    [x,dx] = subFun2(T,P,t,idx);
else
    [x,dx,ddx] = subFun3(T,P,t,idx);
end

end  %END MAIN FUNCTION



%%%% Subfunction for a single output (function value)
function x = subFun1(T,P,t,idx)

x = zeros(size(t));
for i=1:(length(T)-1)
    if sum(idx==i)>0
        dt = (T(i+1)-T(i));
        tt = (t(idx==i)-T(i))/dt;
        p = P(1,:);
        dp = P(2,:)*dt;
        ddp = P(3,:)*dt*dt;
        x(idx==i) = ...
            autoGen_pwPoly5(tt,...
            p(i),dp(i),ddp(i),...
            p(i+1),dp(i+1),ddp(i+1));
    end
end

end


%%%% Subfunction for two outputs (function value and slope)
function [x,dx] = subFun2(T,P,t,idx)

x = zeros(size(t));
dx = zeros(size(t));
for i=1:(length(T)-1)
    if sum(idx==i)>0
        dt = (T(i+1)-T(i));
        tt = (t(idx==i)-T(i))/dt;
        p = P(1,:);
        dp = P(2,:)*dt;
        ddp = P(3,:)*dt*dt;
        [x(idx==i), dx(idx==i)] = ...
            autoGen_pwPoly5(tt,...
            p(i),dp(i),ddp(i),...
            p(i+1),dp(i+1),ddp(i+1));
    end
end

end


%%%% Subfunction for two outputs (function value and slope)
function [x,dx,ddx] = subFun3(T,P,t,idx)

x = zeros(size(t));
dx = zeros(size(t));
ddx = zeros(size(t));
for i=1:(length(T)-1)
    if sum(idx==i)>0
        dt = (T(i+1)-T(i));
        tt = (t(idx==i)-T(i))/dt;
        p = P(1,:);
        dp = P(2,:)*dt;
        ddp = P(3,:)*dt*dt;
        [x(idx==i), dx(idx==i), ddx(idx==i)] = ...
            autoGen_pwPoly5(tt,...
            p(i),dp(i),ddp(i),...
            p(i+1),dp(i+1),ddp(i+1));
    end
end

end

