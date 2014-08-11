function data = simulate_slideNeg(setup)

%This function simulates the stick sliding to the left 

%Set up the integration algorithm
Tspan = setup.Tspan;
IC = [  setup.IC.th;
    setup.IC.x;
    setup.IC.dth;
    setup.IC.dx];
eventFunc = @(t,z)events_slideNeg(t,z,setup.P);
options = odeset(...
    'RelTol',setup.tol,...
    'AbsTol',setup.tol,...
    'Vectorized','on',...
    'MaxStep',setup.odeMaxStep,...
    'Events',eventFunc);
userfun = @(t,z)dynamics_slideNeg(t,z,setup.P);

%Run simulation
sol = feval(setup.solver,userfun,Tspan,IC,options);

%Format for post processing
tspan = [sol.x(1),sol.x(end)];
nTime = ceil(setup.dataFreq*diff(tspan));
t = linspace(tspan(1),tspan(2),nTime);
Z = deval(sol,t);
[~, C, E] = dynamics_slideNeg(t,Z,setup.P);

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

%Get transitions for finite state machine
data.phase = 'SLIDE_NEG';
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

