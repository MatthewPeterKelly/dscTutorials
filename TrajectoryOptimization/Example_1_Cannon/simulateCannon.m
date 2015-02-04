function traj = simulateCannon(init,param)

v0 = init.speed;
th0 = init.angle;

c = param.c;  %Quadratic drag coefficient
nGrid = param.nGrid;

% Set up initial conditions for ode45
x0 = 0;  y0 = 0;  %Start at the origin
dx0 = v0*cos(th0);
dy0 = v0*sin(th0);
if dy0 < 0, error('Cannot point cannon through ground! sin(th0) > 0 Required.'); end;

% Set up arguments for ode45
userFun = @(t,z)cannonDynamics(t,z,c);  %Dynamics function
tSpan = [0,100];  %Never plan on reaching final time
z0 = [x0;y0;dx0;dy0];  %Initial condition
options = odeset('Events',@groundEvent,'Vectorized','on');

% Run a simulation
sol = ode45(userFun, tSpan, z0, options);

% Extract the solution on uniform grid:
traj.t = linspace(sol.x(1), sol.x(end), nGrid);
z = deval(sol,traj.t);
traj.x = z(1,:); 
traj.y = z(2,:); 
traj.dx = z(3,:); 
traj.dy = z(4,:); 

end