function dz = rhs(~,z,P)
%Wrapper function for ode45

q = z(1:3,:);
dq = z(4:6,:);
u = controller(q,dq,P);
ddq = rocketDynamics(0,q,dq,u,P);

dz = [dq;ddq];

end