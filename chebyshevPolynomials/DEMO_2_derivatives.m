% DEMO  --  Chebyshev Polynomial Interpolation and Differentiation
% UPDATED  --  October 28, 2013
% Written by Matthew Kelly, Cornell University
%
% This script demonstrates how chebyshev interpolants can be used to get
% accurate derivate information. 
%

%% General Settings
clear; clc;

%What order should the approximation be?
order = 34;  %Low = 25, %Med = 1000, %High = 100000

%How many points should be used for error calculations and plotting?
nTime = 1000;

%What domain should we be looking at?
d = [0,1.3];

%Time for use in plots
time = linspace(d(1),d(2),nTime);

% %The following version of time will yield more accurate results and a
% %faster runtime because the points in time do not line up exactly with
% the chebyshev grid points.
% time = linspace(d(1)-1e-4, d(2)+2e-4,nTime);  

%% DEMO  --  Fit to analytic function

%Set up analytic function
IO.domain = d;
IO.userFunc = @testFunction;

%Get the values at each of the chebyshev nodes
f = chebyshevFit(IO, order);         %Chebyshev Values
tic
[y,Dy,DDy,DDDy] = chebyshevInterpolate(f,time,d);   %Approximation
toc

%get the exact (analytic) solution for each derivative
[g,Dg,DDg,DDDg] = testFunction(time);

%Show results
figure(401); clf;  
subplot(4,2,1);
    plot(time,y,'b-','LineWidth',2) 
    title(['function approximation - order ' num2str(order)]);
subplot(4,2,3)
    plot(time,Dy,'b-','LineWidth',2) 
    title(['derivative approximation - order ' num2str(order)]);
subplot(4,2,5)
    plot(time,DDy,'b-','LineWidth',2) 
    title(['second derivative approximation - order ' num2str(order)]);
subplot(4,2,7)
    plot(time,DDDy,'b-','LineWidth',2) 
    title(['third derivative approximation - order ' num2str(order)]);
subplot(4,2,2);
    semilogy(time,abs(g-y)) 
    title('error in function');
subplot(4,2,4)
    semilogy(time,abs(Dg-Dy)) 
    title('error in derivative');
subplot(4,2,6)
    semilogy(time,abs(DDg-DDy)) 
    title('error in second derivative');
subplot(4,2,8)
    semilogy(time,abs(DDDg-DDDy)) 
    title('error in third derivative');

