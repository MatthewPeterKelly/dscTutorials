% DEMO  --  Chebyshev Polynomial Node Derivatives
% UPDATED  --  October 28, 2013
% Written by Matthew Kelly, Cornell University
%
% This demo is designed to show the accuracy of computing derivatives
% exactly on the chebyshev nodes. This is useful for using chebyshev
% interpolants to solve ordinary differential equations and for trajectory
% optimization using collocation. 
%

%% General Settings
clear; clc;

%What order should the approximation be?
order = 100;

%What domain should we be looking at?5
d = [0,1];

%Get node values
IO.domain = d;
IO.userFunc = @testFunction;
f = chebyshevFit(IO, order);         %Chebyshev Values

%Get derivative values
[Df, DDf, DDDf] = chebyshevDerivative(f,d);

%Get nodes (for plotting):
x = chebyshevPoints(length(f),d);

%Get analytic solution:
t = linspace(d(1),d(2),1000);
[g, Dg, DDg, DDDg] = testFunction(x);

%plot:
figure(402); clf;
subplot(3,1,1); hold on
    semilogy(x,abs(Df-Dg),'k.','MarkerSize',10)
    title(['Error - First Derivative at Nodes - Order ' num2str(order)])
    set(gca,'YScale','log') %Not sure why this is needed here...
subplot(3,1,2); hold on
    semilogy(x,abs(DDf-DDg),'k.','MarkerSize',10)
    title(['Error - Second Derivative at Nodes - Order ' num2str(order)])
    set(gca,'YScale','log') %Not sure why this is needed here...
subplot(3,1,3); hold on
    semilogy(x,abs(DDDf-DDDg),'k.','MarkerSize',10)
    title(['Error - Third Derivative at Nodes - Order ' num2str(order)])
    set(gca,'YScale','log') %Not sure why this is needed here...
