function traj = dirTrans_Euler(config)

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

% Based on the Trapazoid Method for discretization, as defined in Bett's book,
% chapter 4.

n = pack.nState(2);
[t,x,u] = unPackDecVar(z,pack);
dt = t/(n-1);

% Evaluate the dynamics at each collocation point
dx = cartPoleDynamics(x,[u; zeros(size(u))],config.dyn);  

% Trapazoid rule:
idxLow = 1:(n-1);
idxUpp = 2:n;
intStateTrap = 0.5*dt*(dx(:,idxLow) + dx(:,idxUpp));
intStateCol = x(:,idxUpp)-x(:,idxLow);

% Defect constraint:
defect = intStateTrap - intStateCol;

% user-defined boundary constraints:
[bndIneq, bndEq] = boundaryConstraint(t,x(:,1),x(:,end),config.userData);

C = bndIneq;
Ceq = [reshape(defect,numel(defect),1); bndEq];


end


function cost = costFunctionWrapper(z,pack)

[t,x,u] = unPackDecVar(z,pack);

% Trapazoid rule to integrate cost function:
dc = costFunction(x,u);
nTime = size(x,2);
dt = t/(nTime-1);
w = ones(nTime,1); w([1,end]) = 0.5;
cost = dt*dc*w;

end

