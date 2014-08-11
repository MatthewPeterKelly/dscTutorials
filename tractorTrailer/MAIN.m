clear; clc;

%Get all user-defined parameters.
Params = Set_Parameters();

%Create "Trajectory" which describes the road. Since the LQR trajectory
%needs to know the deviation from the target state, the road is described
%as a series of points in state space.
Trajectory = Create_Trajectory(Params);

%Run a simulation - drive the tractor trailer truck along with road, using
%an EKF to do state estimation and a LQR controller to adjust the steering
%angle and speed.
Results = Simulator(Params, Trajectory);

%Unpack the results:
Time = Results.Time;
Goal_Idx = Results.Goal_Idx;
States = Results.States;
Inputs = Results.Inputs;
Trace_Cov = Results.Trace_Cov;
State_Error = Results.State_Error;

%Display things to the user:
Render_Video = false;
if Render_Video
    Create_Video(States,Params,Trajectory, Goal_Idx)
else
    Animate_System(States,Params,Trajectory, Goal_Idx)
end
  %  PLOT_Map(States,Params,Trajectory)
    PLOT_Inputs(Time,Inputs)
    %PLOT_Stop_Action(States,Params,Trajectory)
    PLOT_Estimator(Time, Trace_Cov, State_Error);
