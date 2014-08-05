%MAIN  --  Bouncing Ball Simulator
%UPDATED --  October 18, 2013
%
% This script uses ode45 with events to simulate a ball bouncing over an
% uneven surface.
%

clc; clear;

%Get all of the user defined parameters
P = Set_Parameters();

%% Run the simulation

    %The results of the simulation are stored here. Each cell contains a
    %singel ode45 output struct
    Trajectory = cell(P.maxBounce,1);
    
    %Initialize the loop:
        IC = P.initCond;            %State at the start of each trajectory
        tNow = 0;                   %time at the start of each trajectory
        tEnd = P.maxTime;           %end simulation at this time
        maxBounce = P.maxBounce;    %end simulation if this many bounces
        dynFunc = @(t,X)Ball_Dynamics(t,X,P);   %Handle for the dynamics function
        TermCond = 'Termination Condition: Max Bounces';  %For display only
        
    %Loop through each section of the trajectory
    for bounceNum=1:maxBounce
        %Run the dynamics until bounce or out of time
        Tspan = [tNow,tEnd];         
        sol = ode45(dynFunc,Tspan,IC,P.Options); %events defined in Options
        
        %Store the state immediately before the collision
        impactState = sol.y(:,end);
        
        %Solve the collision 
        [IC, ballRolling]= impactMap(impactState,P);   %This becomes the new start state 
        
        tNow = sol.x(end);  %Grab the last time
        Trajectory{bounceNum}=sol;  %Store the output of ode45
        
        %Check exit condition
        if tEnd <= tNow
            TermCond = 'Termination Condition: Max Time';
            break
        elseif ballRolling
            TermCond = 'Termination Condition: Ball started rolling';
            break
        end
    end
    disp(TermCond);  %Display the termination condition

    %Remove any extra cells in the trajectory
    Trajectory = Trajectory(1:bounceNum);

    
%% Prepare the data for plotting and animation:
% This section interpolates the solution returned by ode45 to a much finer
% grid that is used for plotting and animation.
   
    %It is typically bad practice to allocate memory by just appending new
    %blocks to the arrays. Here it does not cost us too much time, and it
    %makes the code a bit easier to read and understand.
        timeAll = [];   %Stores the interpolated solution for plotting
        stateAll = [];
        timeOde = [];   %Stores the original ode45 solution, stitched
        stateOde = [];
    %Loop through data, interpolating and then stitching:
    for i=1:length(Trajectory)

        %Figure out where we want to interpolate the data
        dt = P.plotTimeStep;
        tStart = Trajectory{i}.x(1);
        tFinal = Trajectory{i}.x(end);
        time = linspace(tStart,tFinal,round((tFinal-tStart)/dt));

        if ~isempty(time)
            %Now use fancy interpolation to create evenly spaced points
            state = deval(Trajectory{i},time);
        else
            disp('Something funny happened - only one grid point in this trajectory section')
            time = Trajectory{i}.x;
            state = Trajectory{i}.y;
        end
        
        %Make sure that the ball didn't pass through the ground and print a
        %warning and suggested fix if it does.
        heightList = EventFunction([],state);
        if min(heightList)<-P.Options.AbsTol
            %Then the ball passed through the ground
            disp('WARNING - the ball passed through the ground!')
            disp('    Suggested fix - reduce the MaxStep size in P.Options by editing Set_Parameters.m')
        end

        %Append new data to the matricies
        timeAll = [timeAll, time]; %#ok<AGROW>
        stateAll = [stateAll, state]; %#ok<AGROW>
        timeOde = [timeOde, Trajectory{i}.x]; %#ok<AGROW>
        stateOde = [stateOde, Trajectory{i}.y]; %#ok<AGROW>
    
    end

%Plot States
figure(1);
PlotStates(timeAll, stateAll, P);

%Plot Energy
figure(2);
PlotEnergy(timeAll, stateAll, P);

%Animate the solution
figure(3);
Animate(timeAll, stateAll, stateOde, P);

