% DEMO  --  Chebyshev Polynomial ODE Solver - order vs accuracy
% UPDATED  --  October 28, 2013
% Written by Matthew Kelly, Cornell University
%
%
% This script demonstrates how the accuracy of the Chebyshev ode solver
% is dependant on the order of the underlying polynomial. The 'truth' is
% created by matlab's ode45 solution (with the tolerances set to be almost
% as small as possible). 
%
% RESULTS:
%   If the order is high enough to capture the shape of the function, then
%   the accuracy is limited by the machine precision in the underlying
%   optimization algoirthm - roughly 1e-14 on my computer.
% 
%   If the order is too low to capture the function, then this method will
%   return very strange results, thus it is always wise to do a convergence
%   check. Maybe in the future I will build this into a future version of
%   the chebyshev ode solver
%
%   ode45 is faster for low-accuracy solutions
%   chebyshevODEsolve is faster for high-accuracy solutions
%
%


%% General Settings
clear; clc;

%What order should the approximations be?
orderRange = [2,200];
orderNum = 25;
orderList = unique(round(logspace(...
    log10(orderRange(1)),...
    log10(orderRange(2)),...
    orderNum)));  

%What domain should we be looking at?
tspan = [0,1];

%Set up the data logging:
N = length(orderList);
cpuTime = zeros(N,1);
maxError = zeros(N,2);
meanError = zeros(N,2);

%Parameters:
Input.order = [];   %Fill in later
Input.tspan = tspan;
z0 = [0.25;0];
Input.z0 = z0;

P.m = 0.1;
P.c = 0.3;
P.k = 100;
userFunc = @(t,z)simpleHarmonicOscillator(z,P);
Input.userFunc = userFunc;


%Get the ode45 solution:
Options = odeset(...
    'RelTol',2.5e-14,...   %ode45 complains if reltol is much smaller
    'AbsTol',2.5e-14);
tic
sol = ode45(userFunc,tspan,z0,Options);
ode45_runTime = toc;

%Run the calculations:
for i=1:N
    
    %Solve the ode at this order
    disp(['Order: ' num2str(orderList(i))])
    tic;
        Input.order = orderList(i);
        zSoln = chebyshevODEsolve(Input);
        f = zSoln(1,:);
        Df = zSoln(2,:);
    cpuTime(i) = toc;

    %Calculate the error using accurate interpolation of ode45 solution
    time = chebyshevPoints(orderList(i)+1,tspan);
    zOde45 = deval(sol,time);
    g = zOde45(1,:);
    Dg = zOde45(2,:);
    
    %Log the error for plotting later
    e = abs(g-f);
    De = abs(Dg-Df);
    maxError(i,:) = [max(e),max(De)];
    meanError(i,:) = [mean(e),mean(De)];
end

%Display the ode45 run time to demonstrate how the timing for the solution
%compares to the chebyshev solution (shown on a plot)
disp(['ODE45 run time: ' num2str(ode45_runTime)])

%% Plots:

figure(406); clf;
subplot(3,2,1)
    loglog(orderList,cpuTime)
    title('CPU Time')
    xlabel('Order of Chebyshev Polynomial')
    ylabel('Time (s)')
subplot(3,2,2)
    plot(time,g)
    title('Function')
    xlabel('Input')
    ylabel('Output')
subplot(3,2,3)
    loglog(orderList,maxError(:,1))
    title('Max Error in Function')
    xlabel('Order of Chebyshev Polynomial')
subplot(3,2,5)
    loglog(orderList,meanError(:,1))
    title('Mean Error in Function')
    xlabel('Order of Chebyshev Polynomial')
subplot(3,2,4)
    loglog(orderList,maxError(:,2))
    title('Max Error in Derivative')
    xlabel('Order of Chebyshev Polynomial')
subplot(3,2,6)
    loglog(orderList,meanError(:,2))
    title('Mean Error in Derivative')
    xlabel('Order of Chebyshev Polynomial')
