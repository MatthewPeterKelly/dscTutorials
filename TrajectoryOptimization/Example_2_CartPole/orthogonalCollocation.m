function traj = orthogonalCollocation(config)

n = config.nGrid;

% Create the initial guess:
guess.duration = config.guess.time(end);
d = [0,guess.duration];
[guess.time, weights] = chebyshevPoints(n,d);
guess.state = interp1(...
    config.guess.time', config.guess.state', guess.time')';
guess.control = interp1(...
    config.guess.time', config.guess.control', guess.time')';

% Pack up guess:
[zGuess,pack] = packDecVar(guess.duration,guess.state,guess.control);
nDecVar = length(zGuess);

%%%% Build linear constraints:
[tIdx,xIdx,uIdx] = getIndex(pack);

% Boundary conditions:
Aeq = zeros(4+4, nDecVar);
beq = zeros(4+4, 1);
for i=1:4
Aeq(i, xIdx.init(i)) = 1;
beq(i) = config.bounds.initialState(i);
end
for i=1:4
Aeq(4+i, xIdx.final(i)) = 1;
beq(4+i) = config.bounds.finalState(i);
end

% State and control bounds:
lb = zeros(nDecVar,1);
lb(tIdx) = config.bounds.duration.lower;
lb(xIdx.x1) = config.bounds.state.lower(1);
lb(xIdx.x2) = config.bounds.state.lower(2);
lb(xIdx.x3) = config.bounds.state.lower(3);
lb(xIdx.x4) = config.bounds.state.lower(4);
lb(uIdx.u1) = config.bounds.control.lower(1);
lb(uIdx.u2) = config.bounds.control.lower(2);
ub = zeros(nDecVar,1);
ub(tIdx) = config.bounds.duration.upper;
ub(xIdx.x1) = config.bounds.state.upper(1);
ub(xIdx.x2) = config.bounds.state.upper(2);
ub(xIdx.x3) = config.bounds.state.upper(3);
ub(xIdx.x4) = config.bounds.state.upper(4);
ub(uIdx.u1) = config.bounds.control.upper(1);
ub(uIdx.u2) = config.bounds.control.upper(2);

% Build the problem struct:
problem.objective = @(z)( costFunctionWrapper(z,pack,weights) ); 
problem.x0 = zGuess;
problem.Aineq = [];
problem.bineq = [];
problem.Aeq = Aeq;
problem.beq = beq;
problem.lb = lb;
problem.ub = ub;
problem.nonlcon = @(z)( nonLinCon(z,pack, config.dyn) );
problem.options = config.options.nlp;
problem.solver = 'fmincon';

% Solve using fmincon:
[zSoln,fSoln,exitFlag] = fmincon(problem);

% Post-processing:
[t,x,u] = unPackDecVar(zSoln,pack);

traj.time = chebyshevPoints(n,[0,t]);
traj.state = x;
traj.control = u;
traj.objVal = fSoln;
traj.exitFlag = exitFlag;
traj.interp = @(zz,tt)( chebyshevInterpolate(zz,tt,[0,t]) );

end


function [C, Ceq] = nonLinCon(z,pack,dyn)

[t,x,u] = unPackDecVar(z,pack);

dz = cartPoleDynamics(x,u,dyn);  

dzTraj = chebyshevDerivative(x,[0,t]);

defect = dz-dzTraj;

C = [];
Ceq = reshape(defect,numel(defect),1);

end


function [t,x,u] = unPackDecVar(z,pack)

nState = pack.nState;
nControl = pack.nControl;
nTime = pack.nTime;

ns = nState*nTime;
nc = nControl*nTime;

iState = 1:ns;
iControl = 1:nc;

t = z(1);
x = reshape(z(1+iState),nState,nTime);
u = reshape(z(1+iState(end)+iControl),nControl,nTime);

end


function [z,pack] = packDecVar(t,x,u)

pack.nState = size(x,1);
pack.nControl = size(u,1);
pack.nTime = size(x,2);

ns = numel(x);
nc = numel(u);

z = [t;reshape(x,ns,1);reshape(u,nc,1)];

end

function [tIdx,xIdx,uIdx] = getIndex(pack)

% This function returns the indices to extract decision variables for use
% in linear constraints to be passed to fmincon.

nDecVar = pack.nTime*(pack.nState+pack.nControl)+1;
[tIdx,x,u] = unPackDecVar(1:nDecVar,pack);

xIdx.init = x(:,1);
xIdx.final = x(:,end);
xIdx.x1 = x(1,:);
xIdx.x2 = x(2,:);
xIdx.x3 = x(3,:);
xIdx.x4 = x(4,:);

uIdx.u1 = u(1,:);
uIdx.u2 = u(2,:);

end


function cost = costFunctionWrapper(z,pack,w)

[t,x,u] = unPackDecVar(z,pack);

% Clenshaw-Curtis Quadrature for integral cost function:
cost = dot(w,costFunction(t,x,u));

end
