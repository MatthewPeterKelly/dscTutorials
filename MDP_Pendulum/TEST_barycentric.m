% MAIN.m
%
% This script is used to test the barycentric interpolation functions. The
% function barycentricWeights is used to compute the weights that should be
% used to represent a given point in n-dimensional state space. The
% function barycentricInterpolate is used to compute the output of a
% function, given the barycentric weights produced in
% barycentricInterpolate.
%
% This particular implementation of barycentric interpoaltion is based on
% Kuhn Triangulation, as described in the paper:
% "Multidimensional Triangulation and Interpolation for
% Reinforcement Learning" by Scott Davies, NIPS 1996
%
% These functions are written in matlab to be easy to read, but are
% intended to be used after being compiled to mex functions.
%

clc; clear;

flagMex = false;
% mex -build barycentricWeights.prj
% mex -build barycentricInterpolate.prj

%%%% ONE DIMENSION: %%%%
xBnd = [-pi,pi];
N = 15;
X = linspace(xBnd(1,1),xBnd(1,2),N(1))';
testFun1 = @(a) ( cos(a) );
Y = testFun1(X);

%Plot data:
figure(1); clf; hold on;
plot(X,Y,'o')

%Randomly sample points between data:
nTest = 500;
R = rand(1,nTest);
x1Test = xBnd(1,1) + (xBnd(1,2)-xBnd(1,1))*R(1,:);
if flagMex
    [W,Idx] = barycentricWeights_mex(x1Test,xBnd,N);
    yTest = barycentricInterpolate_mex(reshape(Y,numel(Y),1),Idx,W);
else %Use matlab version
    [W,Idx] = barycentricWeights(x1Test,xBnd,N);
    yTest = barycentricInterpolate(reshape(Y,numel(Y),1),Idx,W);
end
plot(x1Test,yTest,'.')
xlabel('x1')
ylabel('y')


%%%% TWO DIMENSIONS: %%%%
xBnd = [-1,2; -pi,pi];
N = [5;9];
xLin{1} = linspace(xBnd(1,1),xBnd(1,2),N(1));
xLin{2} = linspace(xBnd(2,1),xBnd(2,2),N(2));
X = cell(2,1);
[X{1}, X{2}] = ndgrid(xLin{1},xLin{2});
testFun2 = @(a,b) ( a.*cos(b) - b.*sin(2*a) );
Y = testFun2(X{1},X{2});

%Plot data:
figure(2); clf; hold on;
x1 = reshape(X{1},numel(X{1}),1);
x2 = reshape(X{2},numel(X{2}),1);
y = reshape(Y,numel(Y),1);
s = 40*ones(size(y)); %MarkerSize
c = y;  %Color!
scatter3(x1,x2,y,s,c,'filled'); view(3);

%Randomly sample points between data:
nTest = 500;
R = rand(2,nTest);
x1Test = xBnd(1,1) + (xBnd(1,2)-xBnd(1,1))*R(1,:);
x2Test = xBnd(2,1) + (xBnd(2,2)-xBnd(2,1))*R(2,:);
if flagMex
    [W,Idx] = barycentricWeights_mex([x1Test;x2Test],xBnd,N);
    yTest = barycentricInterpolate_mex(reshape(Y,numel(Y),1),Idx,W);
else
    [W,Idx] = barycentricWeights([x1Test;x2Test],xBnd,N);
    yTest = barycentricInterpolate(reshape(Y,numel(Y),1),Idx,W);
end
s = 40*ones(size(yTest)); %MarkerSize
c = yTest;  %Color!
scatter3(x1Test,x2Test,yTest,s,c); view(3);
xlabel('x1')
ylabel('x2')
zlabel('y')


%%%% THREE DIMENSIONS: %%%%%
xBnd = [...
    -1,2;
    -pi,pi;
    2,7];
N = [8;6;7];
xLin{1} = linspace(xBnd(1,1),xBnd(1,2),N(1));
xLin{2} = linspace(xBnd(2,1),xBnd(2,2),N(2));
xLin{3} = linspace(xBnd(3,1),xBnd(3,2),N(3));
X = cell(3,1);
[X{1}, X{2}, X{3}] = ndgrid(xLin{1},xLin{2},xLin{3});
testFun3 = @(a,b,c) ( a.*cos(b) - b.*sin(2*a) );
Y = testFun3(X{1},X{2},X{3});

%Plot data:
figure(3); clf; hold on;
x1 = reshape(X{1},numel(X{1}),1);
x2 = reshape(X{2},numel(X{2}),1);
x3 = reshape(X{3},numel(X{3}),1);
y = reshape(Y,numel(Y),1);
s = 40*ones(size(y)); %MarkerSize
scatter3(x1,x2,x3,s,y,'filled'); view(3);

%Randomly sample points between data:
nTest = 1000;
R = rand(3,nTest);
x1Test = xBnd(1,1) + (xBnd(1,2)-xBnd(1,1))*R(1,:);
x2Test = xBnd(2,1) + (xBnd(2,2)-xBnd(2,1))*R(2,:);
x3Test = xBnd(3,1) + (xBnd(3,2)-xBnd(3,1))*R(3,:);
tic
if flagMex
    [W,Idx] = barycentricWeights_mex([x1Test;x2Test;x3Test],xBnd,N);
    yTest = barycentricInterpolate_mex(reshape(Y,numel(Y),1),Idx,W);
else
    [W,Idx] = barycentricWeights([x1Test;x2Test;x3Test],xBnd,N);
    yTest = barycentricInterpolate(reshape(Y,numel(Y),1),Idx,W);
end
toc
ySoln = testFun3(x1Test,x2Test,x3Test)';

s = 40*ones(size(yTest)); %MarkerSize
c = yTest;  %Color!
scatter3(x1Test,x2Test,x3Test,s,c); view(3);
xlabel('x1')
ylabel('x2')
zlabel('x3')
title(['color = y,  mse = ' num2str(mean((yTest-ySoln).^2))])
