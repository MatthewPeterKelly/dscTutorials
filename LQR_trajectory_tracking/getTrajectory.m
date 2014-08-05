function [T,Nom] = getTrajectory(P)

%This function creates a dummy example of a trajectory to track;

N = P.N;   %Number of samples
NN = P.NN;  %Exact trajectory

xMax = P.xMax;
vMax = P.vMax;
uMax = P.uMax;

%Get trajectory
userFunc = @(t,z)rhs(t,z,P);
Tspan = P.Tspan;
IC = P.IC;
sol = ode45(userFunc,Tspan,IC);

%Extract points to linearize about
t = linspace(Tspan(1),Tspan(2),N);
Z = deval(sol,t);
u = control(t,Z,P);

%Store full linear system at each point
T = cell(N,0);
Q = diag(1./[xMax,vMax].^2);
R = 1/uMax^2;
for i=1:N
    T(i).t = t(i);
    T(i).x = Z(:,i);
    T(i).u = u(i);
    [T(i).A, T(i).B] = linearDyn(t(i),Z(:,i),u(i),P);
    T(i).R = R;
    T(i).Q = Q;
end

Nom.t = linspace(Tspan(1),Tspan(2),NN);
Nom.Z = deval(sol,Nom.t);
Nom.u = control(Nom.t,Nom.Z,P);

end

function dZ = rhs(t,z,P)

u = control(t,z,P);
dZ = dynamics(t,z,u,P);

end

function u = control(t,z,P)

w = P.w;  
u = sin(t.*w);

end