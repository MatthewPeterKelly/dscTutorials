function [bounds, userData] = computeBounds(config)
% bounds = computeBounds(config)
%
% This function computes the bounds for the state and control

% Unpack physical parameters
m1 = config.dyn.m1;
% m2 = config.dyn.m2;
g = config.dyn.g;
l = config.dyn.l;

% Boundry values
userData.initialState.lower = [0;0;0;0];
userData.initialState.upper = userData.initialState.lower;
userData.finalState.lower = [config.guess.state(1:2,end);0;0];
userData.finalState.upper = userData.finalState.lower;

% Bounds on duration (total trajectory time)
wn = sqrt(l/g);
bounds.initialTime.lower = 0;
bounds.initialTime.upper = 0;
bounds.finalTime.lower = 0.1*(2*pi*wn);
bounds.finalTime.upper = 10*(2*pi*wn);

% Bounds on state:
bounds.state.lower = [-2*l; -2*pi; -10*l/wn; -10*pi/wn];
bounds.state.upper = [2*l; 2*pi; 10*l/wn; 10*pi/wn];

% Bounds on control:
bounds.control.lower = -4*m1*g;
bounds.control.upper = 4*m1*g;

end