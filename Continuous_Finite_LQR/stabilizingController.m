function u = stabilizingController(t,x,y,xFit,yFit,uFit,kxFit,kyFit)

%This function is a controller that is used to stabilize the system to a
%nominal trajectory.

xRef = polyval(xFit,t);
yRef = polyval(yFit,t);
uRef = polyval(uFit,t);
Kx = polyval(kxFit,t);
Ky = polyval(kyFit,t);

u = uRef - Kx.*(x-xRef) - Ky.*(y-yRef);

end