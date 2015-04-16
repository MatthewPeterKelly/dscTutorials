% DEMO  --  Chebyshev Polynomial Multivariate Interpolation
% UPDATED  --  October 28, 2013
% Written by Matthew Kelly, Cornell University
%
% This script demonstrates how to approximate a multivariable function with
% a chebyshev polynomial interpolant. Here we use it to approximate the
% position and velocity of a simple harmonic oscillator.
%

%What order chebyshev polynomial to use?
order = 25;

%set up the time span for the system integration using ode45
tspan = [0,1];
nTime = 1001;

%Initial conditions for the simple harmonic oscillator
z0 = [0.25;0];   %[Position, Velocity]

%Physical parameters for the simple harmonic osciallator
P.m = 0.1;
P.c = 0.4;
P.k = 100;

%Function handle for the simple harmonic oscillator
userFunc = @(t,z)simpleHarmonicOscillator(z,P);

%Use ode45 to get the solution
Options = odeset(...
    'RelTol',1e-10,...   %ode45 complains if reltol is much smaller
    'AbsTol',1e-10);
sol = ode45(userFunc,tspan,z0,Options);

%Get the "exact" solution at each point in tspan
time = linspace(tspan(1),tspan(2),nTime);
z = deval(sol,time);
pos = z(1,:);
vel = z(2,:);

%Plot ode45 stuff
figure(404); clf
subplot(2,1,1); hold on
    plot(time,pos,'r-','LineWidth',2)
    title('Position')
    xlabel('Time')
    ylabel('Position')
subplot(2,1,2); hold on
    plot(time,vel,'r-','LineWidth',2)
    title('Velocity')
    xlabel('Time')
    ylabel('Velocity')

%% Chebyshev Stuff

IO.input = time;
IO.output = z;

%Check the grid points
[f,d,x] = chebyshevFit(IO, order);
subplot(2,1,1);
    plot(x,f(1,:),'ko','MarkerSize',7,'LineWidth',1)
subplot(2,1,2); 
    plot(x,f(2,:),'ko','MarkerSize',7,'LineWidth',1)

%Check the interpolation
tic
y = chebyshevInterpolate(f,time,d);
toc
posCheck = y(1,:);
velCheck = y(2,:);

%Plots for the chebyshev interpolant
subplot(2,1,1);
    plot(time,posCheck,'k:','LineWidth',1)
    legend('ode45','chebPts','cheb-pos')
subplot(2,1,2); 
    plot(time,velCheck,'k:','LineWidth',1)
legend('ode45','chebPts','cheb-vel')



