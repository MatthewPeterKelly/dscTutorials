function PLOT_Stop_Action(States,Params,Trajectory)

% This function is used to plot the "stop action" version of the simulation.
% This is analgous to running the simulation slowly with the "hold on"
% feature set.

Fig_Num = 4;
N_Frames = 20;


Npts = Params.Traj.Npts;

x = States(1,:);
y = States(2,:);
th = States(3,:);
phi = States(4,:);

Lt = Params.Dyn.Lt;
Lc = Params.Dyn.Lc;

Nx = length(x);

Display = linspace(1,Nx,N_Frames);
Display = round(Display);

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

All_Points = [Pa,Pb,Pc,Road];

Xmin = min(All_Points(1,:));
Xmax = max(All_Points(1,:));
Ymin = min(All_Points(2,:));
Ymax = max(All_Points(2,:));

figure(Fig_Num)
clf;
hold on
axis([Xmin,Xmax,Ymin,Ymax])
axis equal
title('Simulation of the Tractor Trailer Truck')
plot(Road(1,:),Road(2,:),'k','LineWidth',Params.Sim.RoadWidth)
    
for i=Display
    Trailer_X = [Pa(1,i); Pb(1,i)];
    Trailer_Y = [Pa(2,i); Pb(2,i)];
    Cab_X = [Pb(1,i); Pc(1,i)];
    Cab_Y = [Pb(2,i); Pc(2,i)];
    
    plot(Trailer_X,Trailer_Y,'b','LineWidth',20)
    plot(Cab_X, Cab_Y,'g','LineWidth',17)
end

end