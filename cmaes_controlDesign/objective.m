function val = objective(guess,IC)

%Runs a large number of simulations of the controlled simulation and then
%returns a combination of state and actuation error

duration = 3;  
dt = 0.05;
n = round(duration/dt);

z = IC;

%Weighting between relative cost. 
x_cost = 0.7^-2;
v_cost = 1.5^-2;
u_cost = 1.0^-2;

%Run simulation using RK4 integration in parallel
val = 0; %cost accumulator
for i=1:n
    [k1, u] = rhs(z,guess);
    k2 = rhs(z + 0.5*dt*k1,guess);
    k3 = rhs(z + 0.5*dt*k2,guess);
    k4 = rhs(z + k3,guess);
    
    z = z + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    
    val = val + x_cost*sum(z(1,:).^2);
    val = val + v_cost*sum(z(2,:).^2);
    val = val + u_cost*sum(u.^2);
end

end

function [xdot, u] = rhs(z,k)

u = control(z,k);
xdot = dynamics(z,u);

end
