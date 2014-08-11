function Command = Controller(State, Params, Goal_Idx, Trajectory)

persistent K_Prev

if strcmp(Params.Sim.Direction,'reverse')
    Dir = 2;
else
    Dir = 1;
end
if Goal_Idx < 1
    %Then the at the end of the road. Command a stop
    Exit_Flag = true;
    Goal_Idx = Params.Traj.Npts;   %Select the last point
else
    Exit_Flag = false;
end

Goal = Trajectory.States{Dir,Goal_Idx};
K_New = Trajectory.Gains{Dir,Goal_Idx};

if isempty(K_Prev)
    K_Prev = K_New;
end

%Smooth controller switching
alpha = Params.Ctl.Gain_Smoothing;
K = K_New*(1-alpha) + K_Prev*(alpha);

%Determine the Command value
Command = -K*(State-Goal);

%Command a stop if at the end of the road
if Exit_Flag
    Command(1) = 0;
end

K_Prev = K;

end