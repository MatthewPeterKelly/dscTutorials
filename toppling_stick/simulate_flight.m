function data = simulate_flight(setup)

%This function simulates the flight phase of the dynamics

%Set up the integration algorithm
Tspan = setup.Tspan;
IC = [  setup.IC.th;
    setup.IC.x;
    setup.IC.y;
    setup.IC.dth;
    setup.IC.dx;
    setup.IC.dy];
eventFunc = @(t,z)events_flight(t,z,setup.P);
options = odeset(...
    'RelTol',setup.tol,...
    'AbsTol',setup.tol,...
    'Vectorized','on',...
    'MaxStep',setup.odeMaxStep,...
    'Events',eventFunc);
userfun = @(t,z)dynamics_flight(t,z,setup.P);

%Run simulation
sol = feval(setup.solver,userfun,Tspan,IC,options);

%Format for post processing
tspan = [sol.x(1),sol.x(end)];
nTime = ceil(setup.dataFreq*diff(tspan));
t = linspace(tspan(1),tspan(2),nTime);
Z = deval(sol,t);
[~, E] = dynamics_flight(t,Z,setup.P);

%Store in a nice format for plotting
data.time = t;
data.state.th = Z(1,:);
data.state.x = Z(2,:);
data.state.y = Z(3,:);
data.state.dth = Z(4,:);
data.state.dx = Z(5,:);
data.state.dy = Z(6,:);
data.contact.h = zeros(1,nTime);
data.contact.v = zeros(1,nTime);
data.energy.potential = E(1,:);
data.energy.kinetic = E(2,:);
data.P = setup.P;

%Get transitions for finite state machine
data.phase = 'FLIGHT';
if isempty(sol.ie)
    data.exit = 'TIMEOUT';
else
    switch sol.ie(end)
        case 1   %close end of the stick hit the ground
            data.exit = 'STRIKE_0';
        case 2   %Far end of the stick hit the ground
            data.exit = 'STRIKE_2';
        otherwise
            error('Invalid exit condition in simulate flight')
    end
end

end

