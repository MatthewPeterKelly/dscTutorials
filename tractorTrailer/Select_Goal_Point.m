function Goal_Idx = Select_Goal_Point(State,Params,Trajectory)

th = State(3);

if strcmp(Params.Sim.Direction,'reverse')
    Current_Direction = -[-sin(th); cos(th)];
    Dir = 2;
else
    Current_Direction = [-sin(th); cos(th)];
    Dir = 1;
end

Npts = Params.Traj.Npts;
Idx = 1:Npts;

%Determine the relative position vector for all points in Trajectory
Rel_Pos = zeros(2,Npts);
for i=1:Npts
    Rel_Pos(1,i) = Trajectory.States{Dir,i}(1) - State(1); %Relative X position
    Rel_Pos(2,i) = Trajectory.States{Dir,i}(2) - State(2); %Relative Y position
end

%[Index,Distance,Dot_Product]
Data = zeros(Npts,3);



Data(:,1) = Idx';
Data(:,2) = sqrt(Rel_Pos(1,:).^2 + Rel_Pos(2,:).^2);
Data(:,3) = Dot(Current_Direction,Rel_Pos)./Data(:,2)';   %==Cos(heading)

Wrong_Way = Data(:,3) < cos(Params.Ctl.Max_Target_Angle);
Too_Close = Data(:,2) < Params.Ctl.Minimum_Goal_Distance;
End_of_Road = Data(:,1) > (Params.Traj.Npts - Params.Traj.End_Buffer);

N_remove = sum(Wrong_Way | Too_Close | End_of_Road);

if N_remove < Npts   %Then at least one valid point remains
    Data(Wrong_Way | Too_Close | End_of_Road,:) = [];   %Remove invalid points
    Data_Sorted = sortrows(Data,2);
    %Select the closest valid goal point:
    Goal_Idx = Data_Sorted(1,1);
else
    N_backwards = sum(Wrong_Way);
    if N_backwards < Npts   %Then we can just pick a point that is too close
        %Data(Wrong_Way,:) = [];   %Remove invalid points
        %Data_Sorted = sortrows(Data,2);
        %Select the closest valid goal point:
        %Goal_Idx = Data_Sorted(1,1);
        Goal_Idx = -1;   %Delimiter to show that the road is ending
    else  %No valid points. Give up and select the closest
        %Data_Sorted = sortrows(Data,2);
        %Select the closest valid goal point:
        Goal_Idx = -1;   %Delimiter to show that the road is ended
    end
end

end