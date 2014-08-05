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
% demo - attempt to use vandermonde interpolation for same data
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
order = 100; %Low = 10, Med = 100, high = 1000;
%
%%%

%How many points to use when sampleing data from the test function?
nData = 1000;

%How many points should be used for error calculations and plotting?
nTime = 1000;

%What domain should we be looking at?
d = [0,1];

%% Shared variables:

%Sample the function to get a data set
    data.input = linspace(d(1),d(2),nData);
    data.output = testFunction(data.input);
    
%Chebyshev Points for plotting purposes:    
    chebPts = chebyshevPoints(order+1,d);   

%Time for use in plots
    time = linspace(d(1),d(2),nTime);

%Analytic solution at values in time:
    g = testFunction(time);

%% DEMO  --  Fit to analytic function

IO.domain = d;
IO.userFunc = @testFunction;

tic
f1 = chebyshevFit(IO, order);         %Chebyshev Values
y1 = chebyshevInterpolate(f1,time,d);   %Approximation
e1 = g-y1;                              %Error
disp(['Chebyshev analytic fit and interpolate time: ' num2str(toc)]);

%Show results
figure(400); clf;  
subplot(3,2,1); hold on
    plot(chebPts,f1,'k.','MarkerSize',10)   
    plot(time,g,'r-') 
    plot(time,y1,'b-')    
    legend('Chebyshev Nodes','Analytic','Interpolant','Location','NorthWest')
    title(['Chebyshev - Analytic Fit - order ' num2str(order)]);
subplot(3,2,2)
    plot(time,e1)
    title('error')
    

%% DEMO  --  Fit to data

tic
f2 = chebyshevFit(data, order);         %Chebyshev Values
y2 = chebyshevInterpolate(f2,time,d);   %Approximation
e2 = g-y2;                              %Error
disp(['Chebyshev data fit and interpolate time: ' num2str(toc)]);

%Show results
subplot(3,2,3); hold on
    plot(chebPts,f2,'k.','MarkerSize',10)   
    plot(time,g,'r-') 
    plot(time,y2,'b-')    
    legend('Chebyshev Nodes','Analytic','Interpolant','Location','NorthWest')
    title(['Chebyshev - Data Fit - order ' num2str(order)]);
subplot(3,2,4)
    plot(time,e2)
    title('error')
    

%% DEMO  --  Compare to vandermonde interpolation
% This will generate a warning if the order is large enough. This is
% expected, and part of this demonstration.

%Matlab polyfit and polyval uses vandermonde interpolation
tic
p = polyfit(data.input,data.output,order);
y3 = polyval(p,time);                   %Approximation
e3 = g-y3;                              %Error
disp(['Vandermonde data fit and interpolate time: ' num2str(toc)]);

%Show results
subplot(3,2,5); hold on
    plot(time,g,'r-') 
    plot(time,y3,'b-')    
    legend('Analytic','Interpolant','Location','NorthWest')
    title(['Vandermonde - Data Fit - order ' num2str(order)]);
subplot(3,2,6)
    plot(time,e3)
    title('error')
    


