function PLOT_Map(States,Params, Trajectory)

%This function plots an empty version of the map.

x = States(1,:);
y = States(2,:);
th = States(3,:);
phi = States(4,:);

Lt = Params.Dyn.Lt;
Lc = Params.Dyn.Lc;

Npts = Params.Traj.Npts;

%[dStates, A, B, Positions] = Derive_EoM()

Pa = [...   %Position of the back of the trailer
    x;
    y];

Pb = [...    %Position of the back of the cab
 x - Lt*sin(th);
 y + Lt*cos(th)];

Pc = [...   %Position of the front of the cab
 x - Lc*(cos(phi).*sin(th) + cos(th).*sin(phi)) - Lt*sin(th);
 y + Lc*(cos(phi).*cos(th) - sin(phi).*sin(th)) + Lt*cos(th)];

if strcmp(Params.Sim.Direction,'reverse')
    Dir = 2;
else
    Dir = 1;
end
Road = zeros(2,Npts);
for i=1:Npts
    Road(:,i) = Trajectory.States{Dir,i}(1:2);
end

figure(1)
clf; hold on;
plot(Pa(1,1),Pa(2,1),'r','LineWidth',5)
plot(Pb(1,1),Pb(2,1),'b','LineWidth',3)
plot(Pc(1,1),Pc(2,1),'g','LineWidth',3)
legend('Back of Trailer','Back of Cab', 'Front of Cab')
plot(Road(1,:),Road(2,:),'k','LineWidth',Params.Sim.RoadWidth)
title('Map of the Road and Traces')
axis equal

end