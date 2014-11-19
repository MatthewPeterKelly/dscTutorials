function dz = rhs(~,z,u)
%This function is a wrapper for the automatically generated dynamics
%function. It is designed to be called inside of ode45

x = z(1,:); y = z(2,:);
[dx,dy] = dynamics(x,y,u);
dz = [dx;dy];
end