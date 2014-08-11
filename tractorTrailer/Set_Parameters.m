function Params = Set_Parameters()

%% Trajectory 
% Things describing the road - many settings are in Trajectory_Examples.m

Params.Traj.Npts = 100;

%How many points to skip at the end when runing the simulation. This is
%needed because the controller is "looking ahead" while driving.
Params.Traj.End_Buffer = 5;   

%The following lines of code are used to load sample trajectories.
Example_Number = 7; %#ok<NASGU>
Trajectory_Examples



%% Simulator
%  settings that are used in the simulator and plotting

Params.Sim.Tspan = [0,Simulation_Duration]; %seconds
Params.Sim.Nsteps = ceil(diff(Params.Sim.Tspan)*Control_Frequency);  
Params.Sim.Direction = 'reverse';   %Drive 'forward' or 'reverse'
%Params.Sim.Direction = 'forward';   %Drive 'forward' or 'reverse'
Params.Sim.RoadWidth = 80;
Params.Sim.Frames_Per_Second = 24;
Params.Sim.Speed_Up_Video = 10;   %How many times faster than real time
Time = linspace(Params.Sim.Tspan(1),Params.Sim.Tspan(2),Params.Sim.Nsteps);
dt = Time(2)-Time(1);
Params.Sim.dt = dt;
Params.Sim.Time = Time;


%% Estimator
%   An extended kalman filter is used for state estimation. This section
%   contains various parameters for estimation.

x_var = 0.5;   %meters
y_var = 0.5;   %meters
phi_var = (pi/180)*(4);   %radians
Params.Est.Sensor_Noise_Variance = [x_var; y_var; phi_var];
Params.Est.R = diag(Params.Est.Sensor_Noise_Variance.^2);

dx_var = 0.03;  %(m/s) process error in velocity x
dy_var = 0.03;  %(m/s) process error in velocity y
dth_var = (pi/180)*(1);   %(rad/s) process error in orientation
dphi = (pi/180)*(2);   %(rad/s) process error in cab angle
Process_Noise_Variance = [dx_var; dy_var; dth_var; dphi];
Params.Est.Q = diag(Process_Noise_Variance.^2);   

x0_err = 0.1;   %(m) error in initial guess
y0_err = 0.1;   %(m) error in initial guess
th0_err = (pi/180)*2;    %(rad)   error in initial guess
phi0_err = (pi/180)*4;     %(rad) error in initial guess
Xhat0_Err = randn(4,1).*[x0_err; y0_err; th0_err; phi0_err];
Params.Est.Xhat0_Err = Xhat0_Err;
Params.Est.P0 = diag(Xhat0_Err.^2);

%The sensors are: [x;y;phi]
Params.Est.H = [...
      1 0 0 0 ;
      0 1 0 0 ;
      0 0 0 1 ];  



%% Control
%    A linear quadratic regulator is used to provide steering commands.
%    Various parameters, saturation limits, and weightings are set here.

Params.Ctl.Gain_Smoothing = 0.6; %  K = Knew*(1-a) + Kold*(a); a is on [0,1]
Params.Ctl.Steering_Smoothing = 0.6;  %Same as above

Max_Speed = 2;
Max_Steer = (pi/180)*60;

    Params.Ctl.Actuator_Limits = [...
        -Max_Speed,Max_Speed    %Speed limits
        -Max_Steer, Max_Steer];   %Turning Limits

% % Min_Speed = 0.25;
% % if strcmp('reverse',Params.Sim.Direction)
% %     Params.Ctl.Actuator_Limits = [...
% %         -Max_Speed,-Min_Speed    %Speed limits
% %         -Max_Steer, Max_Steer];   %Turning Limits
% % else  
% %     Params.Ctl.Actuator_Limits = [...
% %         Min_Speed,Max_Speed;    %Speed limits
% %         -Max_Steer, Max_Steer];   %Turning Limits
% % end

Max_Turning_Rate = Max_Steer/2;   %(rad/s) roughly 2 seconds to max out steering
Max_Acceleration = Max_Speed/4;   %(m/s) roughly 2 seconds to reach slow running speed
Params.Ctl.Rate_Limits = [...
    -Max_Acceleration, Max_Acceleration;
    -Max_Turning_Rate, Max_Turning_Rate];

Params.Ctl.Max_Target_Angle = (pi/180)*45;   %Accept tarets at +- this value off direction of travel

Max_Tol_State_Error = [...
    Max_Pos_Err;   % (m) x position
    Max_Pos_Err;   % (m) y position
    (pi/180)*1e4;  %(rad) trailer angle theta  (Ignore this one)
    (pi/180)*1e4];%30];   %(rad) cab angle phi
Params.Ctl.Q = diag(Max_Tol_State_Error.^(-2));
Max_Tol_Actuator_Effort = [...
    Max_Speed;    %(m/s) cab velocity
    Max_Steer];  %(rad) steering angle
Params.Ctl.R =  diag(Max_Tol_Actuator_Effort.^(-2));

%look ahead distance has been moved to Trajectory examples
%Params.Ctl.Minimum_Goal_Distance = 1; %(m) how far to "look ahead"


%% Dynamics
%    These are the physical parameters of the tractor trailer truck

Params.Dyn.Lt = 15;   %Trailer Length (m)
Params.Dyn.Lc = 4;   %Cab Length (m)

Params.Dyn.Nu = 2;  %Number of inputs
Params.Dyn.Nx = 4;  %Number of states


end