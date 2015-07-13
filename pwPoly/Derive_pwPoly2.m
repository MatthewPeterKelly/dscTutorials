% Derive_pwPoly2.m
%
% This script derives the equations for quadratic interpolation, where the
% function is defined by its endpoint and midpoint values.
%

clc; clear;


syms a b c 'real'  %Coefficients
syms x 'real' %Independant variable
syms yLow yMid yUpp 'real' %Values at endpoints and midpoints

% Assume that:
%   xLow = -1
%   xMid =  0
%   xUpp =  1
%
%   y = a*x*x + b*x + c;

eqnLow = yLow - (a - b + c);  %subs x = -1
eqnMid = yMid - (c);  %subs x =  0
eqnUpp = yUpp - (a + b + c);  %subs x =  1

eqns = [eqnLow; eqnMid; eqnUpp];
vars = [a;b;c];
[A,b] = equationsToMatrix(eqns,vars);
soln = A\b;

fprintf('y = a*x^2 + b*x + c \n\n');
fprintf('y(-1) = yLow \n') 
fprintf('y(0) = yMid \n') 
fprintf('y(1) = yUpp \n\n')
fprintf(['a = ' char(soln(1)) '\n']);
fprintf(['b = ' char(soln(2)) '\n']);
fprintf(['c = ' char(soln(3)) '\n']);
