function data = simulate_slidePos(setup)

%This function simulates the stick sliding to the right

%Set up the integration algorithm
Tspan = setup.Tspan;
IC = [  setup.IC.th;
    setup.IC.x;
    setup.IC.dth;
    setup.IC.dx];
eventFunc = @(t,z)events_slidePos(t,z,setup.P);
options = odeset(...
    'RelTol',setup.tol,...
    'AbsTol',setup.tol,...
    'Vectorized','on',...
    'MaxStep',setup.odeMaxStep,...
    'Events',eventFunc);
userfun = @(t,z)dynamics_slidePos(t,z,setup.P);

%Run simulation
sol = feval(setup.solver,userfun,Tspan,IC,options);

%Format for post processing
tspan = [sol.x(1),sol.x(end)];
nTime = ceil(setup.dataFreq*diff(tspan));
t = linspace(tspan(1),tspan(2),nTime);
Z = deval(sol,t);
[~, C, E] = dynamics_slidePos(t,Z,setup.P);

%Store in a nice format for plotting
data.time = t;
data.state.th = Z(1,:);
data.state.x = Z(2,:);
data.state.dth = Z(3,:);
data.state.dx = Z(4,:);
data.state.y = zeros(1,nTime) + setup.IC.y;
data.state.dy = zeros(1,nTime);
data.contact.h = C(1,:);
data.contact.v = C(2,:);
data.energy.potential = E(1,:);
data.energy.kinetic = E(2,:);
data.P = setup.P;
data.phase = 'SLIDE_POS';

%Get transitions for finite state machine
if isempty(sol.ie)
    data.exit = 'TIMEOUT';
elseif sol.xe(end) ~= sol.x(end);
    %An event was triggered, but it was non-terminal
    data.exit = 'TIMEOUT';
else
    switch sol.ie(end)
        case 1
            data.exit = 'FALL_NEG';
        case 2
            data.exit = 'FALL_POS';
        case 3
            data.exit = 'STUCK';
        case 4
            data.exit = 'LIFT';
        case 5
            data.exit = 'LIFT';
        otherwise
            error('Invalid exit condition in simulate flight')
    end
end

end

