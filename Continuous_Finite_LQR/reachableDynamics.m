function velocity = reachableDynamics(t,~,schemeData)

%This function calculates the closed-loop backwards dynamics, for use in
%calculation of the backwards reachable set.

% FORWARD REACHABLE

xFit = schemeData.fit.x;
yFit = schemeData.fit.y;
uFit = schemeData.fit.u;
kxFit = schemeData.fit.kx;
kyFit = schemeData.fit.ky;

x = schemeData.grid.xs{1};
y = schemeData.grid.xs{2};

u = stabilizingController(t,x,y,xFit,yFit,uFit,kxFit,kyFit);

[dx,dy] = dynamics(x,y,u);

velocity{1} = dx;
velocity{2} = dy;

end

