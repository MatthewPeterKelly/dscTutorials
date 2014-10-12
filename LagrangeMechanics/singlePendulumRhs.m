function dz = singlePendulumRhs(~,z,g,l)
% This function is used inside of ode45 for running a simulation of a
% single pendulum. The first argument (time) is not used.

th = z(1,:);
w = z(2,:);

dth = w;
dw = singlePendulumDynamics(th,g,l);

dz = [dth;dw];

end