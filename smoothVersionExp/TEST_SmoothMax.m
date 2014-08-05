%TEST_SmoothMax(x,alpha)

%There are three demonstrations presented below. Please read the section
%heading for each demo to see the details of each experiment.
%
% Demo 1 - smoothness vs alpha
% Demo 2 - error vs number of non-zero elements
% Demo 3 - error vs length of arrar
%
%
% Written by Matthew Kelly
% October 2013
% Cornell University
%

%% Demo 1
% This demo plots a contour map of the ouput of the SmoothMax function over
% a large number of 2d inputs. This allows the user to see the effect that
% the smoothing parameter alpha has on the solution.

%Grid dimensions
N = 120;
xData = linspace(-2,2,N);
yData = linspace(-2,2,N);

%How many plots to make and what values of alpha to plot
PlotDims = [2,3];
alpha = logspace(-3,2,PlotDims(1)*PlotDims(2));

%Get ready for computation
[xx, yy] = meshgrid(xData,yData);
zz = cell(length(alpha),1);

%Run computation, looping through every single point in the grid
for k=1:length(alpha)
    zz{k} = zeros(size(zzReal));
    for i=1:N
        for j=1:N
            zz{k}(i,j) = SmoothMax([xx(i,j),yy(i,j)], alpha(k));
        end
    end
end

%Plotting
index = 0;
figure(110);clf;
for i=1:PlotDims(1)
    for j=1:PlotDims(2)
        index = index+1;
        subplot(PlotDims(1),PlotDims(2),index)
        contour(xx,yy,zz{index},12,'LineWidth',2);
        title(['alpha = ' num2str(alpha(index))])
        xlabel('x(1)')
        ylabel('y(1)')
    end
end

%% Demo 2 
% This function has the unfortunate property that the error created by the
% smoothing is related to the number of elements that are close to the
% maximum value. 
%
% The setup for this experiment is to create a vector that is entirely ones
% and zeros, like: [1,1,1,0,0,0,0,0,0,0]. Thus, max(x)==1 in all cases.
% This demo sweeps through cases from [1,0,0,...,0,0,0] to [1,1,1,...,1,1]
% for each value of alpha. For small alpha, there is minimal error, but as
% alpha gets large, there is a huge error.
%

xMax = cell(length(alpha),1);
for k=1:length(alpha)
    x = zeros(100,1);
    xMax{k} = zeros(size(x));
    for i=1:length(x)
        x(i) = 1;
        xMax{k}(i) = SmoothMax(x,alpha(k));  %Use the default alpha value == 1;
    end
end

%Plotting
index = 0;
figure(111);clf;
for i=1:PlotDims(1)
    for j=1:PlotDims(2)
        index = index+1;
        subplot(PlotDims(1),PlotDims(2),index)
        plot(xMax{index}-1,'k-','LineWidth',2);
        title(['alpha = ' num2str(alpha(index))])
        xlabel('Number of non-zero elements')
        ylabel('Percent error')
    end
end



%% Demo 3
% In order to be "smooth" the SmoothMax function includes a small portion
% of every single element in the array. Thus, the length of the array has
% an effect on the error in the result.
%
% Each vector in this test starts at 0 and linearly increases to a final
% value of 1. The independant variable is the number of elements in the
% array.
%

xMax = cell(length(alpha),1);
maxLength = 100;
arrayLength = 2:maxLength;
for k=1:length(alpha)
    xMax{k} = zeros(maxLength-1,1);
    for i=1:length(arrayLength)
        x = linspace(0,1,arrayLength(i));
        xMax{k}(i) = SmoothMax(x,alpha(k));  %Use the default alpha value == 1;
    end
end

%Plotting
index = 0;
figure(112);clf;
arrayLength = 2:100;
for i=1:PlotDims(1)
    for j=1:PlotDims(2)
        index = index+1;
        subplot(PlotDims(1),PlotDims(2),index)
        plot(arrayLength,xMax{index}-1,'k-','LineWidth',2);
        title(['alpha = ' num2str(alpha(index))])
        xlabel('length of array')
        ylabel('Percent error')
    end
end
