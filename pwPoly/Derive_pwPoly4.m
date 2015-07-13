% Derive_pwPoly2.m
%
% This script derives the equations for quartic interpolation, where the
% function is defined by its value and slope at the endpoitns, and the
% value at the midpoint
%

clc; clear;


syms a b c d e 'real'  %Coefficients
syms x 'real' %Independant variable
syms yLow dyLow yMid yUpp dyUpp 'real' %Values at endpoints and midpoints

% Assume that:
%   xLow = -1
%   xMid =  0
%   xUpp =  1
%

y = simplify(e + x*(d + x*(c + x*(b + x*a))));
dy = simplify(diff(y,x));

eqnLow = yLow - subs(y,x,sym(-1));
eqnLowD = dyLow - subs(dy,x,sym(-1));
eqnMid = yMid - subs(y,x,sym(0));
eqnUpp = yUpp - subs(y,x,sym(1));
eqnUppD = dyUpp - subs(dy,x,sym(1));

eqns = [eqnLow; eqnLowD; eqnMid; eqnUpp; eqnUppD];
vars = [a;b;c;d;e];
[A,b] = equationsToMatrix(eqns,vars);
soln = simplify(A\b);

disp(['a = ' char(soln(1))]);
disp(['b = ' char(soln(2))]);
disp(['c = ' char(soln(3))]);
disp(['d = ' char(soln(4))]);
disp(['e = ' char(soln(5))]);