function bounds = computeBounds(config)
% bounds = computeBounds(config)
%
% This function returns the bounds on the decision variables, to be passed
% to the trajectory optimization solver.
%
% INPUTS:
%   config.dyn = physical parameters of the model
%       .m1 = cart mass
%       .m2 = pole mass
%       .g = gravity
%       .l = pendulum length
%
% OUTPUTS:
%   bounds = struct of bounds for decision variables:
%       .initialTime.lower = scalar
%       .initialTime.upper = scalar
%       .finalTime.lower = scalar
%       .finalTime.upper = scalar
%       .initialState.lower = [ns, 1]
%       .initialState.upper = [ns, 1]
%       .finalState.lower = [ns, 1]
%       .finalState.upper = [ns, 1]
%       .state.lower = [ns, 1]
%       .state.upper = [ns, 1]
%       .control.lower = [nc, 1]
%       .control.upper = [nc, 1]
%


% Unpack physical parameters
m1 = config.dyn.m1;
% m2 = config.dyn.m2;
g = config.dyn.g;
l = config.dyn.l;

% Boundry values
bounds.initialState.lower = [0;0;0;0];
bounds.initialState.upper = bounds.initialState.lower;
bounds.finalState.lower = [config.guess.state(1:2,end);0;0];
bounds.finalState.upper = bounds.finalState.lower;

% Bounds on duration (total trajectory time)
wn = sqrt(l/g);
bounds.initialTime.lower = 0;
bounds.initialTime.upper = 0;
bounds.finalTime.lower = 0.5*(2*pi*wn);
bounds.finalTime.upper = 4*(2*pi*wn);

% Bounds on state:
bounds.state.lower = [-1.0*l; -2*pi; -10*l/wn; -10*pi/wn];
bounds.state.upper = [1.0*l; 2*pi; 10*l/wn; 10*pi/wn];

% Bounds on control:
bounds.control.lower = -5*m1*g;
bounds.control.upper = 5*m1*g;

end