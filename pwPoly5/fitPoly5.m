function P = fitPoly5(T,t,x)
% P = fitPoly5(T,t,x)
%
% Fits a piece-wise 5th-order polynomial to the function x(t), using the
% knot-points in T
%
% INPUTS:
%     T = [1, nKnot] vector of the time at each knot point
%     t = [1, nTime] = vector of desired interpolation times
%     x = [1, nTime] = function value
%  
%   OUTPUTS:
%     P = [3, nKnot] = [x;dx;ddx] = function data at each knot point
%  
%   ASSUME:
%     --> T is monotonicall increasing: diff(T) > 0
%     --> All inputs contain only real numbers
%     --> T(1) <= min(t) <= T(end)
%
% See Also: pwPoly5 fitPoly5p

%%%% Create a starting guess for optimization:
dt = diff(t);
dx = diff(x)./dt; dx = [dx, dx(end)];  
ddx = diff(dx)./dt; ddx = [ddx, ddx(end)];

xKnot = interp1(t,x,T,'linear','extrap');
dxKnot = interp1(t,dx,T,'linear','extrap');
ddxKnot = interp1(t,ddx,T,'linear','extrap');

Pguess = [xKnot; dxKnot; ddxKnot];

nKnot = length(T);
zGuess = reshape(Pguess,3*nKnot,1);

options = optimoptions('fminunc',...
    'MaxFunEvals',200*length(zGuess),...
    'Display','off',...
    'Algorithm','quasi-newton');

objFun = @(z)checkFit(z,T,t,x,nKnot);
zSoln = fminunc(objFun,zGuess,options);

P = reshape(zSoln,3,nKnot);

end

function mse = checkFit(z,T,t,x,nKnot)

P = reshape(z,3,nKnot);
xFit = pwPoly5(T,P,t);

mse = mean((x-xFit).^2);

end
