function soln = cannon_collocation(guess,target,param)
% soln = cannon_collocation(guess,target,param)
%
% This function uses single shooting to solve the cannon problem.
%
% INPUTS:
%   guess.initSpeed
%   guess.initAngle
%   target.x
%   target.y
%   param.c = quadratic drag coefficient
%   param.nGrid = number of grid points for the integration method
%   param.nSegment = number of trajectory segments
%
% OUTPUTS:
%   soln.t
%   soln.x
%   soln.y
%   soln.dx
%   soln.dy
%

global ITER_LOG_COLLOCATION;  %Used for diagnostics and visualization only
ITER_LOG_COLLOCATION = [];

P.nGrid = param.collocation.nSegment + 1;
P.c = param.dynamics.c;

%%% Run a simulation to get the initial guess:
init.speed = guess.initSpeed;
init.angle = guess.initAngle;
traj = simulateCannon(init,P);
guess.T = traj.t(end);  %Trajectory duration

%%% Break the guess trajectory at segment bounds:
guess.z = [traj.x; traj.y; traj.dx; traj.dy];

%%% Store the initial guess for the problem:
nState = 4;  %Number of states in the problem (x,y,dx,dy)
problem.x0 = [guess.T, reshape(guess.z,1,nState*P.nGrid)];
problem.lb = [];    % Lower bound on decision variables
problem.ub = [];   % Upper bound on decision variables

%%% Set up the linear constraints (there are none);
problem.Aineq = [];  problem.Aeq = [];
problem.bineq = [];  problem.beq = [];

%%% Set up the user-defined functions:
problem.objective = @(decVar)objective(decVar(4),decVar(5));  %Objective (cost) function
problem.nonlcon = @(decVar)nonLinCst(decVar,target,P);   %NonLinear constraints

%%% Set up the options for the solver:
problem.solver = 'fmincon';
if param.diagnostics.enable  %Then record full diagnostics:
    problem.options = optimset(...
        'Display','iter',...
        'MaxFunEvals',1e4,...
        'MaxIter',100,...
        'OutputFcn',@(decVar,optimVal,state)outFun(decVar,optimVal,state));
else %Run things quickly!
    problem.options = optimset(...
        'MaxFunEvals',1e4,...
        'MaxIter',100,...
        'Display','off');
end

%Use FMINCON to solve the constrained optimization problem:
[zSoln, fVal, exitFlag] = fmincon(problem);

%Call the constraint function one final time to get the trajectory:
[~, ~, t, zTraj] = nonLinCst(zSoln,target,P);

%%% Store the trajectory in a nice format:
soln.t = t;
soln.x = zTraj(1,:);
soln.y = zTraj(2,:);
soln.dx = zTraj(3,:);
soln.dy = zTraj(4,:);
soln.success = exitFlag == 1;
soln.cost = fVal;
soln.method = 'Direct Collocation';

%%% Run diagnostics on the solution if desired:
if param.diagnostics.enable
    for i=1:length(ITER_LOG_COLLOCATION)
        decVar = ITER_LOG_COLLOCATION(i).decVar;
        dt = decVar(1)/param.collocation.nSegment;
        z = reshape(decVar(2:end),nState,P.nGrid);
        ITER_LOG_COLLOCATION(i).defect = getDefect(dt,z,P.c);
    end
    diagnostics_collocation(target,param,soln)
else
    figure(param.diagnostics.figNum.collocation); clf;
    plotSoln(soln, target, param);
end

end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%           Non-linear constraint function for collocation                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function [C, Ceq,tTraj,zTraj] = nonLinCst(decVar,target,P)
% This is the key function!
% There are two key constraints here: Boundary Conditions & Dynamics
% The dynamics are using hermite-simpson quadrature between grid points
% The decision variables are time, followed by the states at each
% collocation point.
% There are constraints placed on both the initial and final states
% 
% The details of the method are described in section 4.5 of the book by
% John T. Betts
% Practical Methods for Optimal Control and Estimation Using Nonlinear
% Programming. 2010.
%
% Another good reference is:
% "A Survey of numerical methods for optimal control"
%  - Anil Rao, 2009, Advances in Astronautical Sciences
%

nState = 4;  %Number of states in the problem (x,y,dx,dy)
nGrid = P.nGrid;
tEnd = decVar(1);
z = reshape(decVar(2:end),nState,nGrid);

% Parallel quadrature integration between each pair of collocation points
dt = tEnd/(nGrid-1);
d = getDefect(dt,z,P.c); %Simulate the trajectory

% Compute the boundary conditions:
BoundaryInit = z(1:2, 1); %Initial Position
BoundaryFinal = z(1:2, end) - [target.x; target.y]; %Final Position

% Pack up the constraints:
C = [];  %No inequality constraints
Ceq = [BoundaryInit; reshape(d,4*(nGrid-1),1); BoundaryFinal];

if nargout==4  %Only used for post-processing  --  return trajectory
    tTraj = linspace(0,tEnd,P.nGrid+1);
    zTraj = z;
end

end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Compute the Defects - Direct Collocation                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function d = getDefect(dt,z,c)
%
% Compute defects via hermite simpson quadrature:
%
% f = dynamics(t,z)
%
% d(k) = z(k+1) - z(k) - (dt/6)*(f(k) + 4*fBar(k+1) + f(k+1))
%  
% fBar(k+1) = dynamics(t(k) + dt/2, zBar(k+1))
%
% zBar(k+1) = (1/2)*(z(k)+z(k+1)) + (dt/8)*(f(k) - f(k+1))
%
% Reference:
% "A Survey of numerical methods for optimal control"
%  - Anil Rao, 2009, Advances in Astronautical Sciences
%

%%% Some useful quantities:
zLow = z(:,1:(end-1));
zUpp = z(:,2:end);

%%% Compute the dynamics at each grid point:
f = cannonDynamics([],z,c);  %no time dependence
fLow = f(:,1:(end-1));
fUpp = f(:,2:end);

%%% Compute the cannon dynamics at intermediate point:
zBar = 0.5*(zLow + zUpp) + (dt/8).*(fLow-fUpp);
fBar = cannonDynamics([],zBar,c);

%%% Compute the defects:
d = zUpp - zLow - (dt/6).*(fLow + 4*fBar + fUpp);

end



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Data-logging and Diagnostics:                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


function stop = outFun(decVar,optimVal,state)
% This function is used for logging the progress of fmincon throughout the
% optimization run. It does not affect the optimization process.

global ITER_LOG_COLLOCATION;  %Keeping track of iteration details

stop = false;

switch state
    case 'init'
    case 'iter'
        iter = optimVal.iteration+1;
        ITER_LOG_COLLOCATION(iter).optimVal = optimVal;
        ITER_LOG_COLLOCATION(iter).decVar = decVar;
    case 'done'
    otherwise
end

end


