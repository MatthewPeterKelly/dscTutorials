%DEMO_9_chebEval.m

% This script is designed to test chebEval, a fast version of
% chebyshevInterpolate that is to be used as a compiled mex function.

f = rand(2,10);
d = [0,4];
t = linspace(d(1),d(2),500);

y = chebEval(f,t,d);

plot(t,y);