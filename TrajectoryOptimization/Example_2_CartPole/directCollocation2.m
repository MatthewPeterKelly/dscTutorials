function traj = directCollocation2(config)

n = config.grid.nTrajPts;

% Create the initial guess:
guess.time = linspace(config.guess.time(1),config.guess.time(end),n);
guess.state = interp1(...
    config.guess.time', config.guess.state', guess.time')';
guess.control = interp1(...
    config.guess.time', config.guess.control', guess.time')';

% Create a list of all linear constraints, then add function handles:
[problem, pack] = buildConstraints(guess,config);
problem.objective = @(z)( costFunctionWrapper(z,pack) ); 
problem.nonlcon = @(z)( nonLinCon(z,pack, config) );

% Solve using fmincon:
[zSoln,fSoln,exitFlag] = fmincon(problem);

% Post-processing:
[t,x,u] = unPackDecVar(zSoln,pack);

traj.time = linspace(0,t,n);
traj.state = x;
traj.control = u;
traj.objVal = fSoln;
traj.exitFlag = exitFlag;
traj.interp.state = @(tt)( interp1(traj.time',traj.state',tt')' );
traj.interp.control = @(tt)( interp1(traj.time',traj.control',tt')' );

end

function [C, Ceq] = nonLinCon(z,pack,config)
%
% This function enforces the dynamics of the cart-pole system
%

n = pack.nState(2);
[t,x,u] = unPackDecVar(z,pack);
dt = t/(n-1);

if mod(n,2) ~= 1
error('number of grid-points must be odd for dircol2!');
end

idxLow = 1:2:(n-2);
idxMid = 2:2:(n-1);
idxUpp = 3:2:n;

xLow = x(:,idxLow);
xMid = x(:,idxMid);
xUpp = x(:,idxUpp);
uLow = u(:,idxLow);
uMid = u(:,idxMid);
uUpp = u(:,idxUpp);

%Simpson's Rule:
dxLow = cartPoleDynamics(xLow,[uLow; zeros(size(uLow))],config.dyn);
dxMid = cartPoleDynamics(xMid,[uMid; zeros(size(uMid))],config.dyn);
dxUpp = cartPoleDynamics(xUpp,[uUpp; zeros(size(uUpp))],config.dyn);
xUppStar = xLow + (dt/6)*(dxLow + 4*dxMid + dxUpp);

% Defect constraint:
defect = xUpp - xUppStar;

% user-defined boundary constraints:
[bndIneq, bndEq] = boundaryConstraint(t,x(:,1),x(:,end),config.userData);

C = bndIneq;
Ceq = [reshape(defect,numel(defect),1); bndEq];

end


function cost = costFunctionWrapper(z,pack)


error('BUG HERE: too many degrees of freedom in u')
% This method has no been set up properly. Need to rewrite.



n = pack.nState(2);
[t,x,u] = unPackDecVar(z,pack);
dt = t/(n-1);

if mod(n,2) ~= 1
error('number of grid-points must be odd for dircol2!');
end

idxLow = 1:2:(n-2);
idxMid = 2:2:(n-1);
idxUpp = 3:2:n;

xLow = x(:,idxLow);
xMid = x(:,idxMid);
xUpp = x(:,idxUpp);
uLow = u(:,idxLow);
uMid = u(:,idxMid);
uUpp = u(:,idxUpp);

%Simpson's Rule:
dcLow = costFunction(xLow,uLow);
dcMid = costFunction(xMid,uMid);
dcUpp = costFunction(xUpp,uUpp);
cost = sum((dt/6)*(dcLow + 4*dcMid + dcUpp));

end






