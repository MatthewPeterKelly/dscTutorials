% MAIN - River Crossing
%
% Problem Statement:
%
% A boat travels across a river with a constant speed. The water is flowing
% in the river with a parabolic velocity profile, with zero speed at the
% banks. The boat can choose its direction of travel at any instant. What
% is the path that crosses the river in minimum time?
%
% Transcription Method:
%
% Direct Collocation - Trapazoid Integration
% 

%%% Problem Parameters:
param.maxWaterSpeed = 0.1;   %(m/s)
param.riverWidth = 10.0;  %(m)
param.boatSpeed = 2.0;  %(m/s)

%%% Transcription Parameters:
param.nGrid = 30;  % Number of grid points to use for dynamics

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Set up problem                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% State: z = [x;y] = [distance along river; distance across river];
% Control u = boat heading (absolute angle);
%

%%% Bounds on the time to cross the river:
bound.timeLow = 1.0;
bound.timeUpp = inf;

%%% Bounds on state  (Boat must stay in river)
bound.stateLow = [-inf; 0];
bound.stateUpp = [inf; param.riverWidth];

%%% Bounds on control
bound.controlLow = 0;   %Straight down-stream
bound.controlUpp = pi;  %Straight up-stream

%%% Initial guess - assume that we cross in a straight line
xGuess = [0,0];
yGuess = linspace(0,param.riverWidth,2);
guess.time = [0, 2.0*param.riverWidth/param.boatSpeed];
guess.state = [xGuess; yGuess];
guess.control = (pi/2)*[1,1];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Solve and plot Solution                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

[time, state, control, exitFlag, output] = ...
    directCollocation_trapazoid(bound, guess, param);

figure(1); clf;

subplot(3,1,1);
plot(time,state(1,:),'LineWidth',2)
ylabel('x');

subplot(3,1,2);
plot(time,state(2,:),'LineWidth',2);
ylabel('y');

subplot(3,1,3);
plot(time, control,'LineWidth',2);
ylabel('u')
xlabel('t');

figure(2); clf;

plot(state(1,:),state(2,:));
xlabel('x - distance along bank')
ylabel('y - distance across river')
axis equal;




