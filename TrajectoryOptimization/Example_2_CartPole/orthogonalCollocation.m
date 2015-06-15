function traj = orthogonalCollocation(config)

n = config.grid.nTrajPts;

% Create the initial guess:
guess.duration = config.guess.time(end);
d = [0,guess.duration];
[guess.time, weights] = chebyshevPoints(n,d);
guess.state = interp1(...
    config.guess.time', config.guess.state', guess.time')';
guess.control = interp1(...
    config.guess.time', config.guess.control', guess.time')';

% Create a list of all linear constraints, then add function handles:
[problem, pack] = buildConstraints(guess,config);
problem.objective = @(z)( costFunctionWrapper(z,pack,weights) ); 
problem.nonlcon = @(z)( nonLinCon(z,pack, config) );

% Solve using fmincon:
[zSoln,fSoln,exitFlag] = fmincon(problem);

% Post-processing:
[t,x,u] = unPackDecVar(zSoln,pack);

traj.time = chebyshevPoints(n,[0,t]);
traj.state = x;
traj.control = u;
traj.objVal = fSoln;
traj.exitFlag = exitFlag;
traj.interp.state = @(tt)( chebyshevInterpolate(x,tt,[0,t]) );
traj.interp.control = @(tt)( chebyshevInterpolate(u,tt,[0,t]) );

end


function [C, Ceq] = nonLinCon(z,pack,config)

[t,x,u] = unPackDecVar(z,pack);

dz = cartPoleDynamics(x,[u; zeros(size(u))],config.dyn);  

dzTraj = chebyshevDerivative(x,[0,t]);

defect = dz-dzTraj;

% user-defined boundary constraints:
[bndIneq, bndEq] = boundaryConstraint(t,x(:,1),x(:,end),config.userData);

C = bndIneq;
Ceq = [reshape(defect,numel(defect),1); bndEq];

end

function cost = costFunctionWrapper(z,pack,w)

[t,x,u] = unPackDecVar(z,pack);

% Clenshaw-Curtis Quadrature for integral cost function:
cost = dot(w,costFunction(x,u));

end
