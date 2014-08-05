% DEMO  --  Chebyshev Polynomial Interpolation Order vs Accuracy
% UPDATED  --  October 28, 2013
% Written by Matthew Kelly, Cornell University
%
%
% This script demonstrates how the accuracy of a chebyshev approximation is
% dependant on the order of the underlying polynomial.
%
% RESULTS:
%   The accuracy is very bad if the order is too small, but then rapidly
%   improves until it reaches machine precision. The accuracy of the
%   derivative will get slowly worse for very large order apprimations due
%   to the compounding of rounding errors in the matrix multiplication.
%


%% General Settings
clear; clc;

%What order should the approximation be?
orderRange = [2,1000];
orderNum = 20;
orderList = unique(round(logspace(...
    log10(orderRange(1)),...
    log10(orderRange(2)),...
    orderNum)));  

%How many points should be used for error calculations and plotting?
nTime = 1000;

%What domain should we be looking at?
d = [-2,1];

%Time for use in plots
time = linspace(d(1),d(2),nTime);

%Set up the input/output struct:
IO.domain = d;
IO.userFunc = @testFunction;

%Set up the data logging:
N = length(orderList);
cpuTime = zeros(N,1);
maxError = zeros(N,2);
meanError = zeros(N,2);

%Run the calculations:
for i=1:N
    disp(['Order: ' num2str(orderList(i))])
    tic;
    f = chebyshevFit(IO, orderList(i)); 
    [y, Dy] = chebyshevInterpolate(f,time,d);
    cpuTime(i) = toc;
    [g,Dg] = testFunction(time);
    e = abs(g-y);
    De = abs(Dg-Dy);
    maxError(i,:) = [max(e),max(De)];
    meanError(i,:) = [mean(e),mean(De)];
end

%Make nice plots:
    figure(403); clf;
    subplot(3,2,1)
        loglog(orderList,cpuTime)
        title('CPU Time')
        xlabel('Order of Chebyshev Polynomial')
        ylabel('Time (s)')
    subplot(3,2,2)
        plot(time,testFunction(time))
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
