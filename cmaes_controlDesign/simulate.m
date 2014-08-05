function [t, z, u] = simulate(gains,z0)

duration = 3;  
dt = 0.01;
n = round(duration/dt);

z = zeros(2,n);

z(:,1) = z0;
t = zeros(1,n);
for i=2:n
    k1 = rhs(z(:,i-1),gains);
    k2 = rhs(z(:,i-1) + 0.5*dt*k1,gains);
    k3 = rhs(z(:,i-1) + 0.5*dt*k2,gains);
    k4 = rhs(z(:,i-1) + k3,gains);
    
    z(:,i) = z(:,i-1) + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    t(i) = t(i-1) + dt;
end

u = control(z,gains);

end

function [xdot, u] = rhs(z,k)

u = control(z,k);
xdot = dynamics(z,u);

end
