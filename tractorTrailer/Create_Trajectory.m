function Trajectory = Create_Trajectory(Params)
%
% FUNCTION: 
%   This function is used to draw the road that the tractor trailer truck
%   is driving on. A different representation is used for the forward and
%   reverse versions of the road. 
%
%   This function defines a trajectory as a series of points and gains
%   associated with that point. Trajectory is a struct with two fields.
%
% INPUT:
%   Params = a struct of parameters for this problem. The field Traj is
%   used in this function. See Set_Parameters.m and Trajectory_Examples.m
%   for more details.
%
% OUTPUT:
%   Trajectory.States = a (2xN) cell array with the state at each of the N points
%   Trajectory.Inputs = a (2xN) cell array the nominal actuator effort
%   Trajectory.Gains = A (2xN) cell array. 
%   {1,:} ==> Forward
%   {2,:} ==> Reverse


Npts = Params.Traj.Npts;

if Params.Traj.Func == false

    %Set constraint points:
    SetPts = Params.Traj.SetPts;
    Order = Params.Traj.Order;

    P = polyfit(SetPts(:,1),SetPts(:,2),Order);
    x = linspace(min(SetPts(:,1)),max(SetPts(:,1)),Npts+1);
    y = polyval(P,x);
else
   x = Params.Traj.x;
   y = Params.Traj.y;
end

%Now we need the desired trailer angle theta, which is defined as the angle
%between the trailer center line and the positive y axis.

%Vector direction between successive points
x_step = diff(x);
y_step = diff(y);
angle = atan2(y_step,x_step);  %angle from the positive x axis
th = angle-pi/2;

%Assume that the cab angle should always try to be centered
phi = zeros(size(th));

%Truncate the vectors
x=x(1:end-1);
y=y(1:end-1);


% % % %PLOT THINGS
% % % 
% % % subplot(2,1,2)
% % % plot(x(1:end-1),th)
% % % title('Angle vs X Position')
% % % 
% % % subplot(2,1,1)
% % % plot(x(1:end-1),y(1:end-1))
% % % axis equal
% % % title('Road')

%% Now do gain matrix calculations

Trajectory.States = cell(2,Npts);
Trajectory.Inputs = cell(2,Npts);
Trajectory.Gains = cell(2,Npts);
for i=1:Npts
   %Drive Forward
        State = [x(i); y(i); th(i); phi(i)];
        Input = [1;0];  %[speed;steering]
        Trajectory.States{1,i} = State;
        Trajectory.Inputs{1,i} = Input;
        Trajectory.Gains{1,i} =  Set_Gain_Matrix(State, Input, Params);   
   
   %Drive Backward
        State_Rev = [x(i); y(i); th(i)+pi; phi(i)];
        Input_Rev = [-1,0];   %[speed;steering]
        Trajectory.States{2,i} = State_Rev;
        Trajectory.Inputs{2,i} = Input_Rev;
        Trajectory.Gains{2,i} =  Set_Gain_Matrix(State_Rev, Input_Rev, Params); 
end


end