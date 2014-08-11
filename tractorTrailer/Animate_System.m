function Animate_System(States,Params,Trajectory,Goal_Idx)

Fig_Num = 2;

N_Frames = Params.Sim.Frames_Per_Second * diff(Params.Sim.Tspan) / Params.Sim.Speed_Up_Video;
Frames = linspace(1,Params.Sim.Nsteps,N_Frames);
Frames = round(Frames);

Time_Target = linspace(1,diff(Params.Sim.Tspan)/Params.Sim.Speed_Up_Video,N_Frames);
TimeStep=0;

Npts = Params.Traj.Npts;

x = States(1,:);
y = States(2,:);
th = States(3,:);
phi = States(4,:);

Lt = Params.Dyn.Lt;
Lc = Params.Dyn.Lc;

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
tic;
for i=Frames
    clf(Fig_Num);
    hold on
    axis([Xmin,Xmax,Ymin,Ymax])
    axis equal
    Trailer_X = [Pa(1,i); Pb(1,i)];
    Trailer_Y = [Pa(2,i); Pb(2,i)];
    Cab_X = [Pb(1,i); Pc(1,i)];
    Cab_Y = [Pb(2,i); Pc(2,i)];
       
    Idx = Goal_Idx(i);
    if Idx < 1
        Idx = Params.Traj.Npts;
        %At the end of the road
    end
    
    Target = Road(:,Idx);
    
    %Plot road
        plot(Road(1,:),Road(2,:),'k','LineWidth',Params.Sim.RoadWidth)
    %Plot the traces:
        plot(Pa(1,1:i),Pa(2,1:i),'b','LineWidth',4)
        plot(Pc(1,1:i),Pc(2,1:i),'g','LineWidth',4)
    %Plot truck
        plot(Trailer_X,Trailer_Y,'b','LineWidth',20)
        plot(Cab_X, Cab_Y,'g','LineWidth',17)
    %Plot the control target
        plot(Target(1),Target(2),'ro','MarkerSize',18,'LineWidth',3)
    title('Simulation of the Tractor Trailer Truck')
    axis equal
    
    TimeStep = TimeStep + 1;
    Time_Error = Time_Target(TimeStep)-toc;
    if Time_Error > 0
        pause(Time_Error)
    else
        pause(0.001)
    end
end




end