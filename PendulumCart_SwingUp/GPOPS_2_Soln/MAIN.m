%---------------------------------------------------%
% Pendulum Cart Swing-Up Problem:                   %
%---------------------------------------------------%

clear; clc; 

%Load solution from file?    ( '' for none )
guessFile = '';%'oldSoln.mat'; %  'oldSoln.mat'  OR   ''

%Physical parameters
auxdata.massBob = 0.2;   %Mass of the pendulum
auxdata.massCart = 1;   %Mass of the cart
auxdata.gravity = 9.81;   %Set negative to have 0 be the unstable equilibrium
auxdata.lengthPendulum = 0.5;   %Length of the pendulum

%Timing parameters:
t0 = 0; 
tfmin = 0; tfmax = 2;

%State parameters [pos; vel; angle; rate];
stateInitial =   [0, 0, pi,   0]; %Initial state [x,v,th,w];
stateFinal =     [0, 0, 0,    0];  %Final state [x,v,th,w];
stateBoundLow = -[1, 10, 3*pi, 4*pi];  %[x,v,th,w]
stateBoundUpp =  [1, 10, 3*pi, 4*pi];  %[x,v,th,w]

%Actuator parameters:
forceBoundLow = -100;    %(N)
forceBoundUpp =  100;    %(N)

%Integral parameters:
integralGuess = 50;
integralBoundLow = 0;
integralBoundUpp = 1000;

%-------------------------------------------------------------------------%
%----------------------- Setup for Problem Bounds ------------------------%
%-------------------------------------------------------------------------%
iphase = 1;
bounds.phase.initialtime.lower = t0; 
bounds.phase.initialtime.upper = t0;
bounds.phase.finaltime.lower = tfmin; 
bounds.phase.finaltime.upper = tfmax;
bounds.phase.initialstate.lower = stateInitial; 
bounds.phase.initialstate.upper = stateInitial; 
bounds.phase.state.lower = stateBoundLow; 
bounds.phase.state.upper = stateBoundUpp; 
bounds.phase.finalstate.lower = stateFinal; 
bounds.phase.finalstate.upper = stateFinal; 
bounds.phase.control.lower = forceBoundLow; 
bounds.phase.control.upper = forceBoundUpp;
bounds.phase.integral.lower = integralBoundLow;
bounds.phase.integral.upper = integralBoundUpp;

%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%

if strcmp(guessFile, '')  %Then use defaults
    guess.phase.time    = [t0; tfmax]; 
    guess.phase.state   = [stateInitial; stateFinal];
    guess.phase.control = [0; 0];
    guess.phase.integral = integralGuess;
else %Load from a data file:
    load(guessFile);
    guess.phase.time = outputPrev.result.solution.phase(1).time;
    guess.phase.state = outputPrev.result.solution.phase(1).state;
    guess.phase.control = outputPrev.result.solution.phase(1).control;
    guess.phase.integral = outputPrev.result.objective;
    
    %Use a mesh that matches with input:
    Npts = length(guess.phase.time);
    nColPts = 4;
    nInterval = floor(Npts/nColPts);
    
    setup.mesh.colpoints = nColPts*ones(1,nInterval);
    setup.mesh.phase(1).fraction = ones(1,nInterval)/nInterval;
    
end

%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%        
%-------------------------------------------------------------------------%
setup.name = 'PendulumCart_SwingUp';
setup.functions.continuous = @pendulumCart_Continuous;
setup.functions.endpoint = @pendulumCart_Endpoint;
setup.auxdata = auxdata;
setup.bounds = bounds;
setup.guess = guess;
setup.nlp.solver = 'ipopt'; % {'ipopt','snopt'}
setup.derivatives.supplier = 'adigator';%'sparseCD';  %'adigator'
setup.derivatives.derivativelevel = 'first';%'second';
setup.mesh.method = 'hp-PattersonRao';
setup.mesh.tolerance = 1e-6;
setup.mesh.maxiteration = 4;
setup.mesh.colpointsmin = 4;
setup.mesh.colpointsmax = 10;
setup.method = 'RPM-Integration';

%-------------------------------------------------------------------------%
%------------------------- Solve Problem Using GPOP2 ---------------------%
%-------------------------------------------------------------------------%
output = gpops2(setup);
solution = output.result.solution;

%--------------------------------------------------------------------------%
%------------------------------- Plot Solution ----------------------------%
%--------------------------------------------------------------------------%

Time = solution.phase(1).time';
State = solution.phase(1).state';
Force = solution.phase(1).control';

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
    plot(Time, Force,'k.','MarkerSize',mSize)
    plot(Time, Force);
    title(['cost: ' num2str(output.result.objective)])
    xlabel('time (s)')
    ylabel('force (N)')
    
% % %Save the solution if desired:
% % outputPrev = output;
% % save('oldSoln.mat','outputPrev');
