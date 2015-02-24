function soln = cannon_gpops(guess,target,param)
% soln = cannon_gpops(guess,target,param)
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

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Set up the problem for gpops                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

setup.bounds.phase.initialtime.lower = 0;
setup.bounds.phase.initialtime.upper = 0;
setup.bounds.phase.finaltime.lower = 0;
setup.bounds.phase.finaltime.upper = inf;
setup.bounds.phase.initialstate.lower = [0,0,-inf, -inf];
setup.bounds.phase.initialstate.upper = [0,0,inf,inf];
setup.bounds.phase.state.lower = -inf(1,4);
setup.bounds.phase.state.upper = inf(1,4);
setup.bounds.phase.finalstate.lower = [target.x,target.y,-inf, -inf];
setup.bounds.phase.finalstate.upper = [target.x,target.y,inf,inf];

setup.guess.phase.time = linspace(0,guess.T,P.nGrid)';
setup.guess.phase.state = guess.z';

setup.auxdata.c = param.dynamics.c;

setup.name = 'cannon';
setup.functions.continuous = @continuous;
setup.functions.endpoint = @endpoint;

setup.nlp.solver = 'snopt'; % {'ipopt','snopt'}
setup.nlp.snoptoptions.maxiterations = 500;

setup.derivatives.supplier = 'sparseCD';
setup.derivatives.derivativelevel = 'first';

setup.mesh.method = 'hp-PattersonRao';
setup.mesh.maxiterations = 6;
setup.mesh.tolerance = 1e-4;
setup.mesh.colpointsmin = 4;
setup.mesh.colpointsmax = 10;

nSegment = param.gpops.nSegment;
gridGuess = ones(1,nSegment);
setup.mesh.phase.fraction = gridGuess/sum(gridGuess);
setup.mesh.phase.colpoints = 4*ones(1,nSegment);

setup.method = 'RPM-Integration';


output = gpops2(setup);


%%% Store the trajectory in a nice format:
soln.t = output.result.interpsolution.phase.time';
soln.x = output.result.interpsolution.phase.state(:,1)';
soln.y = output.result.interpsolution.phase.state(:,2)';
soln.dx = output.result.interpsolution.phase.state(:,3)';
soln.dy = output.result.interpsolution.phase.state(:,4)';
soln.success = output.result.nlpinfo == 1;
soln.cost = output.result.objective;
soln.method = 'GPOPS';


figure(param.diagnostics.figNum.gpops); clf;
plotSoln(soln, target, param);

end


function output = continuous(input)

z = input.phase.state';
c = input.auxdata.c;

output.dynamics = cannonDynamics([],z,c)';

end

function output = endpoint(input)

z0 = input.phase.initialstate;
output.objective = objective(z0(3),z0(4));

end

