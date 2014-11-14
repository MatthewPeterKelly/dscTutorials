% DEMO  --  Chebyshev Polynomial Interpolation
% UPDATED  --  October 28, 2013
% Written by Matthew Kelly, Cornell University
%
% This script demonstrates how to use chebyshev polynomials to approximate
% arbitrary functions, and compares it to vandermonde interpolation. There
% are three sections to this demo:
%
% demo - fit to analytic function
% demo - fit to data
%
% The analytic function fitting using chebyshev polynomials should produce
% the mode accurate fit possible for a given order. Additionally, it will
% be numerically stable for all orders.
%
% The chebyshev fit to data is also numerically stable, with the accuracy
% improving with order. The accuracy is limited by the sampling of the
% data.
%
% The vandermonde fit to data is more accurate than the chebyshev fit to
% data for linearly spaced data, for low order fits. However, it becomes
% numerically unstable for high-order fits, with the error going to
% infinity in some cases. Note that this is the algorithm that is
% implemented by Matlab's polyfit and polyval functions. For large order
% fits, even if the error in the fit is small, the coefficients are poorly
% conditioned, which can cause other problems down the line.
%
% NOTE - If the polynomial order is high then the vandermonde interpolation
% algorithm (polyfit) will issue a warning telling the user that the
% polynomial is badly conditioned. This is expected, and part of the reason
% that the interpolation method for the chebyshev polynomials is better for
% large order approximations.
%


%% General Settings
clear; clc;

%%%%% PLAY WITH THIS SETTING %%%%
%
%What order should the approximation be?
order = 6; %Low = 3, Med = 5, high = 12;
%
%%%

%How many points to use when sampleing data from the test function?
nData = 100;

%What domain should we be looking at?
d = [-1,1];
testFun = @(x) 0.2*x.*sin(1.2*pi*x + 0.2) + 1./(1+exp(-7*x));
noiseAmpl = 0.05;

%% Data:

%Sample the function to get a data set
t = linspace(d(1),d(2),nData);
y = testFun(t);
ySample = y + noiseAmpl*randn(size(y));  %Add some noise to the data

%Chebyshev Points for plotting purposes:
chebPts = chebyshevPoints(order+1,d);


%% DEMO  --  Fit to data


f = chebFitLs(t,ySample,d,order);         %Chebyshev Values
yFit = chebEval(f,t,d);   %Approximation

%Show results
figure(450); clf; hold on;
plot(chebPts,f,'ko','MarkerSize',20,'LineWidth',3)
plot(t,yFit,'k-','LineWidth',3);
plot(t,y,'r-','LineWidth',4)
plot(t,ySample,'b.','MarkerSize',20)
legend('Chebyshev Nodes','Interpolant','Analytic','Data','Location','NorthWest')
title(['Chebyshev - Data Fit - order ' num2str(order)]);


