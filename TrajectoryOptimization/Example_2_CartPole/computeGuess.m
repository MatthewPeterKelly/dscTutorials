function guess = computeGuess(config)
% guess = computeGuess(config)
%
% This function computes an initial guess at the trajectory for swinging up
% the pendulum.

% Unpack physical parameters
m1 = config.dyn.m1;
m2 = config.dyn.m2;
g = config.dyn.g;
l = config.dyn.l;

n = config.nGuess;  %Number of grid points in guess

tEnd = 2*pi*sqrt(l/g);   %Period of a simple pendulum
t = linspace(0,tEnd,n);

% Assume that the cart follows a sinusoidal trajectory, conserving CoM
xAmp = l*m2/(m1+m2);  %CoM calculations...
x = xAmp*sin(2*pi*t/tEnd);
dx = xAmp*(2*pi/tEnd)*cos(2*pi*t/tEnd);
ddx = -xAmp*(2*pi/tEnd)^2*sin(2*pi*t/tEnd);

% Assume that the pendulum goes from bottom to top in one simple motion
q = -sign(xAmp)*pi*t/tEnd;
dq = -sign(xAmp)*pi*ones(size(t))/tEnd;
ddq = zeros(size(t));

% Compute the inverse dynamics to produce this trajectory:
[F,T] = autoGen_cartPoleInvDyn(q,dq,ddq,ddx,m1,m2,g,l);

% Store the result for return:
guess.time = t;
guess.state = [x;q;dx;dq];
guess.control = [F;T];

end