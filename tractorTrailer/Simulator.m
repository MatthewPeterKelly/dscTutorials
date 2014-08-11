function Results = Simulator(Params, Trajectory)

%This function simulates the system as it attempts to track the trajectory
%that is defined in Trajectory.

Nsteps = Params.Sim.Nsteps;

Nx = Params.Dyn.Nx;
Nu = Params.Dyn.Nu;

%Initialize outputs
    Time = Params.Sim.Time;
    States = zeros(Nx,Nsteps);
    Inputs = zeros(Nu,Nsteps);
    Goal_Idx = zeros(1,Nsteps);   %Store which trajectory state is targeted

dt = Params.Sim.dt;          %Set time step
Goal_Idx(1) = 1;

Params.Ctl.dt = dt;

%Which way is the truck driving?
if strcmp(Params.Sim.Direction,'reverse')
    States(:,1) = Trajectory.States{2,1} + Params.Sim.IC_Error;  %Use Reverse Values
    Inputs(:,1) = [Params.Ctl.Actuator_Limits(1,1),0];
else
    States(:,1) = Trajectory.States{1,1} + Params.Sim.IC_Error;  %Use Forward Values
    Inputs(:,1) = [Params.Ctl.Actuator_Limits(1,2),0];
end

%Initialize the Estimator
Xhat0_Err = Params.Est.Xhat0_Err;
Xhat = States(:,1) + Xhat0_Err;
P = Params.Est.P0;

%Keep track of state error
State_Error = zeros(size(States));
Trace_Cov = zeros(Nsteps,1);

State_Error(:,1) =  States(:,1) - Xhat;
Trace_Cov(1) = trace(P);

%Simulate using euler integration, with a ZoH on inputs
for k=1:(Nsteps-1)  
   %Run dynamics from step K to step (K+1)
   dStates = Dynamics(States(:,k), Inputs(:,k), Params);
   States(:,k+1) = States(:,k) + dStates*dt;    
   
   %Measurements of new state (K+1)
   Z = Sensors(States(:,k+1),Params);  %Measure the state
   
   %Call Estimator 
   [Xhat P] = Estimator(Z,Inputs(:,k),Params,Xhat,P);   %Estimate the state
   
   %Find Target on trajectory
   Goal_Idx(k) = Select_Goal_Point(Xhat,Params,Trajectory);
   
   %Run Controller
   Command = Controller(Xhat, Params, Goal_Idx(k), Trajectory);
   
   %Run Actuator Model
   Inputs(:,k+1) = Actuator_Model(Command, Params);
   
   %Record Estimator Performance
        State_Error(:,k+1) =  States(:,k+1) - Xhat;
        Trace_Cov(k+1) = trace(P);
end

Results.Time = Time;
Results.States = States;
Results.Inputs = Inputs;
Results.Goal_Idx = Goal_Idx;
Results.Trace_Cov = Trace_Cov;
Results.State_Error = State_Error;

end