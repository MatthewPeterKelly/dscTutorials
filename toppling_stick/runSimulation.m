function D = runSimulation(setup)
% function D = RUNSIMULATION(setup)
%
%   This function is used to run a finite state machine based simulation of
%   of stick falling over. While it does support a flight phase, it does
%   not yet have impact equations, so the simulation will terminate at the
%   conclusion of the flight phase.
%
% INPUTS:
%   setup = a struct with fields:
%      IC = a struct with the initial values of each state:
%           th = absolute angle of the stick, measured from the +y axis
%           x = the horizontal position of the contact point O
%           y = the vertical position of point O
%           dth = time derivative of th
%           dx = time derivative of x
%           dy = time derivative of y
%       Tspan = [0, Simulation_TimeOutDuration]
%       tol = ode45 integration tolerance
%       dataFreq = samples per second for returned data
%       P = a struct with physical parameters (self consistent units)
%           g = gravity
%           L = distance between contact (O) and CoM (G)
%           FullLength = distance between contact (O) and far end of stick
%           m = mass of the stick
%           u = coefficient of friction
%           I = moment of inertia about CoM
%           
% OUTPUTS:
%   D = a struct with information about the simulation results. Fields:
%       P = setup.P
%       raw = [1 x N] struct array with the data for each phase of motion
%       phase = [1 x N] cell array with the name of each phase of motion
%       code = [1 x N] cell array with the exit code for each phase 
%       data = a struct with data concatenated across all phases
%       Jumps = [1 x N+1] array of the transition times
%       JumpIdx = [1 x N+1] array with the transition indicies
%

%Figure out which phase the simulation starts in
setup = selectPhase(setup); %{'SLIDE','HINGE','FLIGHT'}
phase = setup.phase;
D.P = setup.P;

%Run the finite state machine
N_max = 10;
for i=1:N_max
    switch phase
        case 'HINGE'
            data = simulate_hinge(setup);
        case 'FLIGHT'
            data = simulate_flight(setup);
        case 'SLIDE_POS'
            data = simulate_slidePos(setup);
        case 'SLIDE_NEG'
            data = simulate_slideNeg(setup);
        otherwise
            error('Invalid phase')
    end
    D.raw(i) = data;  
    D.phase{i} = phase;
    D.code{i} = data.exit;
    [phase, setup] = fsm(phase,setup,data);
    if strcmp(phase,'ABORT')
        break;
    end
end

%%%% Post processing and analysis %%%% 

[D.data, D.Jumps, D.JumpIdx] = stitchData(D.raw);

end