function traj = orthogonalCollocation(config)

n = config.grid.nTrajPts;

% Chebyshev points and weights on the default domain
[xx,ww] = chebyshevPoints(n,[-1,1]);
cheb.xx = xx;
cheb.ww = ww;

% Create the initial guess:
tSpan = [config.guess.time(1),config.guess.time(2)];
guess.time = chebyshevScalePoints(xx,ww,tSpan);
guess.state = interp1(...
    config.guess.time',  config.guess.state',  guess.time')';
guess.control = interp1(...
    config.guess.time',  config.guess.control',  guess.time')';

% Create a list of all linear constraints, then add function handles:
[problem, pack] = buildConstraints(guess,config);
func = config.function;
problem.objective = @(z)( costFunctionWrapper(z,pack,func,cheb) ); 
problem.nonlcon = @(z)( nonLinCon(z,pack, func, cheb) );

% Solve using fmincon:
[zSoln,fSoln,exitFlag] = fmincon(problem);

% Post-processing:
[tSpan,x,u] = unPackDecVar(zSoln,pack);

traj.time = chebyshevPoints(n,tSpan);
traj.state = x;
traj.control = u;
traj.objVal = fSoln;
traj.exitFlag = exitFlag;
traj.interp.state = @(tt)( chebyshevInterpolate(x,tt,tSpan) );
traj.interp.control = @(tt)( chebyshevInterpolate(u,tt,tSpan) );

end


function [C, Ceq] = nonLinCon(z,pack,func,cheb)

[tSpan,x,u] = unPackDecVar(z,pack);
t = chebyshevScalePoints(cheb.xx,cheb.ww,tSpan);

dz = func.dynamics(t,x,u);  

dzTraj = chebyshevDerivative(x,tSpan);

defect = dz-dzTraj;

C = [];
Ceq = reshape(defect,numel(defect),1);

end

function cost = costFunctionWrapper(z,pack,func,cheb)

[tSpan,x,u] = unPackDecVar(z,pack);

[t, w] = chebyshevScalePoints(cheb.xx,cheb.ww,tSpan);

c = func.costIntegrand(t,x,u);  %Cost integrand
cost = dot(w,c); % Clenshaw-Curtis Quadrature

end
