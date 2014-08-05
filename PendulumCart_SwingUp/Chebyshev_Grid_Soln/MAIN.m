%MAIN
%
% Finds the optimal trajectory to swing-up a pendulum cart.
% 
% Trajectory is solved on a Chebyshev grid, 
% implicitly approximating the trajectory (both state and actuation) as a 
% Chebyshev polynomial. Defects (for the collocation method) are computed 
% by comparing the state derivatives as computed by two different 
% methods:
%     1) system dynamics (equations of motion)
%     2) direct (analytic) differention of the chebyshev polynomials

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

%Unpack the solution
Xsoln = Results.Xsoln;
State = [P.MS.Start, Xsoln.state, P.MS.Finish];
Time = chebyshevPoints(size(State,2),[0,Xsoln.duration]);

%Interpolate the polynomial to see behavior between grid points
mSize = 10; %How big to make the gridpoints on the plot
domain = [0,Xsoln.duration];
TimeInterp = linspace(domain(1),domain(2),1000);
StateInterp = chebyshevInterpolate(State,TimeInterp,domain);
ForceInterp = chebyshevInterpolate(Xsoln.force,TimeInterp,domain);

%Simple Plots:
figure(1); clf;
subplot(2,1,1); hold on;
    plot(Time, State(1,:),'k.','MarkerSize',mSize)
    plot(TimeInterp, StateInterp(1,:));
    xlabel('time (s)')
    ylabel('position (m)')
    
subplot(2,1,2); hold on;
    plot(Time, State(2,:),'k.','MarkerSize',mSize)
    plot(TimeInterp, StateInterp(2,:));
    xlabel('time (s)')
    ylabel('velocity (m/s)')
    
figure(2); clf;
subplot(2,1,1); hold on;
    plot(Time, State(3,:),'k.','MarkerSize',mSize)
    plot(TimeInterp, StateInterp(3,:));
    xlabel('time (s)')
    ylabel('angle (rad)')
    
subplot(2,1,2); hold on;
    plot(Time, State(4,:),'k.','MarkerSize',mSize)
    plot(TimeInterp, StateInterp(4,:));
    xlabel('time (s)')
    ylabel('rate (rad/s)')
    
figure(3); clf; hold on
    plot(Time, Xsoln.force,'k.','MarkerSize',mSize)
    plot(TimeInterp, ForceInterp);
    title(['cost: ' num2str(Results.output.Fval)])
    xlabel('time (s)')
    ylabel('force (N)')


%%%% HOW TO SAVE THE CURRENT SOLUTION %%%%
%
% It is often useful to work from a previously found trajectory. Once this
% script has run, just save the 'Results' struct to a matlab data file
% (*.mat') and then you will be able to load that solution the next time
% that you run this script (assuming that you have the
% 'Initialize_from_File' flag set properly in Set_Parameters.m).
%
%save('oldSoln.mat','Results');
