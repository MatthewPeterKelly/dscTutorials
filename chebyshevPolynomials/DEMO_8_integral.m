% DEMO  --  Chebyshev Polynomial Integral
% UPDATED  --  October 28, 2013
% Written by Matthew Kelly, Cornell University
%
% This demo is designed to show how to compute integrals of chebyshev
% polynomials.
%
% Results - Note that there are relative large errors between the chebyshev
% approximation of the integral and the analytic integral, BUT the last
% value (which is the value of the definite integral) is very accurate.
% 
% The last point is so accurate because it is the solution to the definite
% integral over the domain using Clenshaw-Curtis quadrature for an order
% "order" accurate polynomial.
%

%% General Settings
clear; clc;

%What order should the approximation be?
order = 50;

%What domain should we be looking at?
d = [0,1];

%Get chebyshev values at the nodes:
[x,w] = chebyshevPoints(order+1,d);
[~,f] = testFunction(x);
If= chebyshevIntegral(f,d);

%Set the time vector
time = linspace(d(1),d(2),10000);
[Ig, g] = testFunction(time);

%Adjust the constants of integration S.T. they all start at zero:
Ig = Ig-Ig(1);

%Interpolate the chebyshev approximations:
y = chebyshevInterpolate(f,time,d);
Iy = chebyshevInterpolate(If,time,d);

%Show results
figure(407); clf;  
subplot(2,2,1); hold on;
    plot(time,g,'b-','LineWidth',2) 
    plot(x,f,'ko','MarkerSize',5);
    title(['function approximation - order ' num2str(order)]);
    legend('Analytic','Cheb. nodes','Location','NorthWest')
subplot(2,2,3); hold on;
    plot(time,Ig,'b-','LineWidth',2) 
    plot(x,If,'ko','MarkerSize',5);
    title(['integral approximation - order ' num2str(order)]);
        legend('Analytic','Cheb. nodes','Location','NorthWest')
subplot(2,2,2);
    semilogy(time,abs(g-y)) 
    title('error in function');
subplot(2,2,4); hold on;
    semilogy(time,abs(Ig-Iy)) 
    semilogy(time(end),abs(Ig(end)-Iy(end)),...
        'bo','MarkerSize',10,'LineWidth',2) 
    title(['error in definite integral: ' num2str(Ig(end)-Iy(end))]);
    set(gca,'yscale','log')
