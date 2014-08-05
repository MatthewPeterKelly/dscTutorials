function zdot = dynamics(z,u)

%Dynamics for a damped inverted pendulum

m = 1.25;
g = 9.81;
l = 0.5;
c = 0.01;  %Damping
I = m*l*l;

x = z(1,:);  % angle
v = z(2,:);  % rate

dx = v;
dv = -(c/I)*v - (m*g*l/I)*sin(-x) + u/I;

zdot = [dx;dv];

end