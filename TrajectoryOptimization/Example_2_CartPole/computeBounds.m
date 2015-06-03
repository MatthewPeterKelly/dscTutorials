function bounds = computeBounds(config)
% bounds = computeBounds(config)
%
% This function computes the bounds for the state and control

% Unpack physical parameters
m1 = config.dyn.m1;
m2 = config.dyn.m2;
g = config.dyn.g;
l = config.dyn.l;

% Boundry values
bounds.initialState = [0;0;0;0];
bounds.finalState = [config.guess.state(1:2,end);0;0];

% Bounds on duration (total trajectory time)
wn = sqrt(l/g);
bounds.duration.lower = 0.1*(2*pi*wn);
bounds.duration.upper = 10*(2*pi*wn);

% Bounds on state:
bounds.state.lower = [-2*l; -2*pi; -10*l/wn; -10*pi/wn];
bounds.state.upper = [2*l; 2*pi; 10*l/wn; 10*pi/wn];

% Bounds on control:
bounds.control.lower = [-4*m1*g; -1e-6];%-4*m2*g*l];
bounds.control.upper = [4*m1*g; 1e-6];%4*m2*g*l];

end