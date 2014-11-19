function [A, B] = getLinTraj(t,xFit,yFit,uFit)

%Linearizes the system about the nominal trajectory. This function is used
%to turn a non-linear trajectory tracking problem into a time-varying
%linear problem.

x = polyval(xFit,t);
y = polyval(yFit,t);
u = polyval(uFit,t);

[A,B] = getLinSys(x,y,u);

end