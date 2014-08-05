function Sim = runSimulation(T,IC,P)

userFunc = @(t,z)rhs(t,z,P);
Tspan = [T(1).t, T(end).t];
sol = ode45(userFunc,Tspan,IC);

%Extract points to linearize about
Sim.t = linspace(Tspan(1),Tspan(2),P.NN);
Sim.Z = deval(sol,Sim.t);

Nu = size(T(1).u,1);
Sim.u = zeros(Nu,P.NN);
for i=1:P.NN
Sim.u(i) = getInput(Sim.t(i),Sim.Z(:,i),P);
end

end

function dZ = rhs(t,Z,P)

u = getInput(t,Z,P);

dZ = dynamics(t,Z,u,P);

end

function u = getInput(t,Z,P)

[K, xNom, uNom] = getGains(P.T,t);
u = K*(Z-xNom) + uNom;

end