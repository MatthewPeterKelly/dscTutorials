function [C, Ceq] = directCollocation1(z,pack,dyn)
%
% This function enforces the dynamics of the cart-pole system
%

n = pack.nColPt;
[t,x,u] = unPackDecVar(z,pack);
dt = t/(n-1);

% Evaluate the dynamics at each collocation point
dx = cartPoleDynamics(x,u,dyn);

% Trapazoid rule:
idxLow = 1:(n-1);
idxUpp = 2:end;
intStateTrap = 0.5*dt*(dx(:,idxLow) + dx(:,idxUpp));
intStateCol = x(:,idxUpp)-x(:idxLow);

% Defect constraint:
defect = intStateTrap - intStateCol;

% Pack up and return:
C = [];
Ceq = reshape(defect,numel(defect),1);

end
