function P = fitPoly5p(T,t,x)
% P = fitPoly5p(T,t,x)
%
% Fits a piece-wise 5th-order polynomial to the a periodic function x(t),
% given the knot points T.
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
%     --> P(T(1)) == P(T(end))  Function is periodic
%
% See Also: pwPoly5, fitpwPoly5

%%%% Create a starting guess for optimization:
dt = diff(t);
dx = diff(x)./dt; dx = [dx, dx(end)];  
ddx = diff(dx)./dt; ddx = [ddx, ddx(end)];

xKnot = interp1(t,x,T,'linear','extrap');
dxKnot = interp1(t,dx,T,'linear','extrap');
ddxKnot = interp1(t,ddx,T,'linear','extrap');

Pguess = [xKnot; dxKnot; ddxKnot];

% Remove periodic end-point:
Pguess(:,1) = 0.5*(Pguess(:,1) + Pguess(:,end));
Pguess(:,end) = [];

nKnot = length(T)-1;
zGuess = reshape(Pguess,3*nKnot,1);

options = optimoptions('fminunc',...
    'MaxFunEvals',200*length(zGuess),...
    'Display','off',...
    'Algorithm','quasi-newton');

objFun = @(z)checkFit(z,T,t,x,nKnot);
zSoln = fminunc(objFun,zGuess,options);

P = reshape(zSoln,3,nKnot);
P = [P,P(:,1)];  %Add periodic endpoint back in

end

function mse = checkFit(z,T,t,x,nKnot)

P = reshape(z,3,nKnot);
P = [P,P(:,1)];  %Add periodic endpoint back in

xFit = pwPoly5(T,P,t);

mse = mean((x-xFit).^2);

end
