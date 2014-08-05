%MAIN
%
% Finds the optimal trajectory to swing-up a pendulum cart.
%
% solves the trajectory on a uniform grid, 
% approximating the trajectory (both state and actuation) as piecewise 
% linear functions. Defects for collocation are computed by comparing the 
% states, at all but the first time step:
%     1) state at (k+1)
%     2) 4th order Runge-Kutta integration step from state at (k)

clc; clear;

%Set up the problem - parameters, initial guess, decision variable bounds
[P, X] = Set_Parameters();
[Xlow, Xupp] = Set_Bounds(P);

%Call fmincon or snopt
if strcmp(P.MS.solver,'snopt')
  Results = wrapper_SNOPT(P,X,Xlow,Xupp);
elseif strcmp(P.MS.solver,'fmincon')
  Results = wrapper_FMINCON(P,X,Xlow,Xupp);
else
  error('Unknown Solver');
end

%Unpack solution:
Xsoln = Results.Xsoln;
State = [P.MS.Start, Xsoln.state, P.MS.Finish];
Time = linspace(0,Xsoln.duration,P.MS.nGrid);
Force = Xsoln.force;

%A few simple plots:

figure(1); clf;
mSize = 10;
subplot(2,1,1); hold on;
    plot(Time, State(1,:),'k.','MarkerSize',mSize)
    plot(Time, State(1,:));
    xlabel('time (s)')
    ylabel('position (m)')
    
subplot(2,1,2); hold on;
    plot(Time, State(2,:),'k.','MarkerSize',mSize)
    plot(Time, State(2,:));
    xlabel('time (s)')
    ylabel('velocity (m/s)')
    
figure(2); clf;
subplot(2,1,1); hold on;
    plot(Time, State(3,:),'k.','MarkerSize',mSize)
    plot(Time, State(3,:));
    xlabel('time (s)')
    ylabel('angle (rad)')
    
subplot(2,1,2); hold on;
    plot(Time, State(4,:),'k.','MarkerSize',mSize)
    plot(Time, State(4,:));
    xlabel('time (s)')
    ylabel('rate (rad/s)')
    
figure(3); clf; hold on
    plot(Time, Xsoln.force,'k.','MarkerSize',mSize)
    plot(Time, Force);
    title(['cost: ' num2str(Results.output.Fval)])
    xlabel('time (s)')
    ylabel('force (N)')

%Animation of the system:
figure(4);
animatePendulumCart;    

%Stop action figure:
figure(5);
stopActionPendulumCart;

    

%%%% HOW TO SAVE THE CURRENT SOLUTION %%%%
%
% It is often useful to work from a previously found trajectory. Once this
% script has run, just save the 'Results' struct to a matlab data file
% (*.mat') and then you will be able to load that solution the next time
% that you run this script (assuming that you have the
% 'Initialize_from_File' flag set properly in Set_Parameters.m).
%
%save('oldSoln.mat','Results');



