% DEMO  --  Chebyshev Polynomial ODE Solver
% UPDATED  --  October 28, 2013
% Written by Matthew Kelly, Cornell University
%
% This script demonstrates how to use the chebyshev ode solver. Assuming
% that you have used enough grid points (order of the chebyshev polynomial)
% to capture the underlying shape of the solution, then this is a very fast
% and accurate integration method. For very accurate solutions it is much
% faster than ode45.
%
% Note that if the order of the chebyshev polynomial is too low than the
% results are very poor. Look at DEMO_Order_vs_Accuracy for more detail.
%

Input.order = 50;   

%Set the timespan for the ode solution and the initial condition
tspan = [0,1];
z0 = [0.25;0];  %[Position; Velocity]

%Pack up into a struct for chebyshevODEsolve
Input.tspan = tspan;
Input.z0 = z0;

%parameters for the simple harmonic oscillator
P.m = 0.03;
P.c = 0.45;
P.k = 100;
userFunc = @(t,z)simpleHarmonicOscillator(z,P);
Input.userFunc = userFunc;

%Solve the problem with Chebyshev:
tic
zSoln = chebyshevODEsolve(Input);
chebyshevTime = toc;
disp(['Chebyshev Time: ' num2str(chebyshevTime) ' seconds']);

%Interpolate the solution with Chebyshev:
Time = linspace(tspan(1),tspan(2),1000);
zInterp = chebyshevInterpolate(zSoln,Time,tspan);
    
%Solve and interpolate the problem with ode45:
Options = odeset(...
    'RelTol',2.5e-14,...
    'AbsTol',2.5e-14);
tic
    sol = ode45(userFunc,tspan,z0,Options);
ode45Time = toc;
disp(['ODE45 Time: ' num2str(ode45Time) ' seconds']);

%Interpolate the ode45 solution
nPlot = 1000;
time = linspace(tspan(1),tspan(2),nPlot);
zOde45 = deval(sol,time);

%Plot the solutions:
    posCh = zInterp(1,:);
    velCh = zInterp(2,:);
    pos45 = zOde45(1,:);
    vel45 = zOde45(2,:);

    figure(405); clf
    subplot(2,2,1); hold on
        plot(Time,posCh,'b-')
        plot(Time,pos45,'r-')
        title('Position')
        xlabel('Time')
        legend('Chebyshev','ode45')
    subplot(2,2,3); hold on
        plot(Time,velCh,'b-')
        plot(Time,vel45,'r-')
        title('Velocity')
        xlabel('Time')
        legend('Chebyshev','ode45')

%Plot the error
    error = abs(zOde45 - zInterp);
    subplot(2,2,2); hold on
        semilogy(Time,error(1,:))
        title('Position Error')
        xlabel('Time')
    subplot(2,2,4); hold on
        semilogy(Time,error(2,:))
        title('Velocity Error')
        xlabel('Time')

