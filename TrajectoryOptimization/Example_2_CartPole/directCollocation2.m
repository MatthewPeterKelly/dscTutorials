function [C, Ceq] = directCollocation2(z,pack,dyn)
%
% This function enforces the dynamics of the cart-pole system
%

n = pack.nColPt;
[t,x,u] = unPackDecVar(z,pack);
dt = t/(n-1);

if mod(n,2) ~= 1
error('number of grid-points must be odd for dircol2!');
end

idxLow = 1:2:(n-2);
idxMid = 2:2:(n-1);
idxUpp = 3:2:end;

xLow = x(:,idxLow);
xMid = x(:,idxMid);
xUpp = x(:,idxUpp);
uLow = u(:,idxLow);
uMid = u(:,idxMid);
uUpp = u(:,idxUpp);

%Simpson's Rule:
dxLow = cartPoleDynamics(xLow,uLow,dyn);
dxMid = cartPoleDynamics(xMid,uMid,dyn);
dxUpp = cartPoleDynamics(xUpp,uUpp,dyn);
xUppStar = xLow + (dt/6)*(dxLow + 4*dxMid + dxUpp);

% Defect constraint:
defect = xUpp - xUppStar;

% Pack up and return:
C = [];
Ceq = reshape(defect,numel(defect),1);

end


