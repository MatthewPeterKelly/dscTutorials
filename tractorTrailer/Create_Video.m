function Create_Video(States,Params,Trajectory,Goal_Idx)

Fig_Num = 5;

Ex_Num = Params.Sim.Example_Number;

FileName = ['Animation_' num2str(Ex_Num)'];

N_Frames = Params.Sim.Frames_Per_Second * diff(Params.Sim.Tspan) / Params.Sim.Speed_Up_Video;
Frames = linspace(1,Params.Sim.Nsteps,N_Frames);
Frames = round(Frames);

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

% FigHandle = figure(Fig_Num);
% set(FigHandle,'Position',[120 128 995 838]);
figure(Fig_Num);
videoObject = VideoWriter(FileName,'MPEG-4');
videoObject.Quality = 96;
videoObject.FrameRate = Params.Sim.Frames_Per_Second;
videoObject.open();
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
    drawnow;

    writeVideo(videoObject,getframe(gcf));
    
end
videoObject.close();


end