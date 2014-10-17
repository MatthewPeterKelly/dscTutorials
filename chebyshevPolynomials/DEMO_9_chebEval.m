%DEMO_9_chebEval.m

% This script is designed to test chebEval, a fast version of
% chebyshevInterpolate that is to be used as a compiled mex function.

f = rand(2,16);
d = [0,4];
t = linspace(d(1),d(2),1000);

if ~exist('chebEval_mex','file')
   coder -build chebyshevMexProject.prj
end

tic
y1 = chebEval(f,t,d);
time = toc;

tic
y2 = chebEval_mex(f,t,d);
mexTime = toc;

subplot(2,1,1)
plot(t,y1);

subplot(2,1,2);
plot(t,y2);

fprintf('Matlab function time: %6.6f ms\n',1000*time);
fprintf('Mex function time: %6.6f ms\n',1000*mexTime);