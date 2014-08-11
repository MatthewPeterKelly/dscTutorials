function data = simulate_hinge(setup)

%This function simulates the hinge phase of the dynamics

%Set up the integration algorithm
Tspan = setup.Tspan;
IC = [setup.IC.th;
    setup.IC.dth];
eventFunc = @(t,z)events_hinge(t,z,setup.P);
options = odeset(...
    'RelTol',setup.tol,...
    'AbsTol',setup.tol,...
    'Vectorized','on',...
    'MaxStep',setup.odeMaxStep,...
    'Events',eventFunc);
userfun = @(t,z)dynamics_hinge(t,z,setup.P);

%Check that the event function is initially satisfied:
eventVal = feval(eventFunc,Tspan(1),IC);
if (sum(eventVal<0)>0) %IC violates constraints
    tspan = [Tspan(1), Tspan(1) + eps];
    nTime = 2;
    t = tspan;
    Z = IC*ones(1,2);
    [~, C, E] = dynamics_hinge(t,Z,setup.P);
    
    triggered = eventVal<0;
    index = 1:length(triggered);
    code = index(triggered);
    %Check that the number of possible transitions is one
    if length(code) >1
        disp('WARNING - multiple possible transitions!');
    end
    
else  %IC satisfied constraints
    
    %Run simulation
    sol = feval(setup.solver,userfun,Tspan,IC,options);
    
    %Format for post processing
    tspan = [sol.x(1),sol.x(end)];
    nTime = ceil(setup.dataFreq*diff(tspan));
    t = linspace(tspan(1),tspan(2),nTime);
    Z = deval(sol,t);
    [~, C, E] = dynamics_hinge(t,Z,setup.P);
    
    code = sol.ie;
    
end

%Store in a nice format for plotting
data.time = t;
data.state.th = Z(1,:);
data.state.dth = Z(2,:);
data.state.x = zeros(1,nTime) + setup.IC.x;
data.state.dx = zeros(1,nTime);
data.state.y = zeros(1,nTime) + setup.IC.y;
data.state.dy = zeros(1,nTime);
data.contact.h = C(1,:);
data.contact.v = C(2,:);
data.energy.potential = E(1,:);
data.energy.kinetic = E(2,:);
data.P = setup.P;

%Get transitions for finite state machine
data.phase = 'HINGE';
if isempty(code)
    data.exit = 'TIMEOUT';
else
    switch code(end)
        case 1
            data.exit = 'FALL_NEG';
        case 2
            data.exit = 'FALL_POS';
        case 3
            data.exit = 'LIFT';
        case 4
            data.exit = 'SLIP_NEG';
        case 5
            data.exit = 'SLIP_POS';
        otherwise
            error('Invalid exit condition in simulate hinge')
    end
end

end


