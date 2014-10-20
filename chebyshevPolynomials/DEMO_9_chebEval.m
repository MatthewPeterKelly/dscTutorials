%DEMO_9_chebEval.m

% This script is designed to test chebEval, a fast version of
% chebyshevInterpolate that is to be used as a compiled mex function.

nCurves = 3;
nChebPts = 12;

f = rand(nCurves,nChebPts);
d = [0,4];  %Domain for the functions
t = linspace(d(1),d(2),1000);
x = chebyshevPoints(nChebPts,d);

%%%% Attempt to ubild the project if the mex function does not exist
if ~exist('chebEval_mex','file')
    coder -build chebyshevMexProject.prj
end

tic
y1 = chebEval(f,t,d);
time = toc;

tic
y2 = chebEval_mex(f,t,d);
mexTime = toc;

%Check that both compute the same result:
error = mean(mean((y1-y2).^2));

if error > 100*eps
    disp('Error - chebEval and chebEval_mex do not produce identical results')
end

figure(1001); clf; hold on;
plot(t,y1,'LineWidth',2);
plot(x,f,'kx','MarkerSize',8,'LineWidth',2);
extents = axis;
for i=1:nChebPts
    plot([1,1]*x(i),extents(3:4),'k:')
end
ylabel('Function value')
xlabel('Function input')

fprintf('Matlab function time: %6.6f ms\n',1000*time);
fprintf('Mex function time: %6.6f ms\n',1000*mexTime);