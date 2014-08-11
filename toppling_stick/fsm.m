function [nextPhase, setup] = fsm(phase,setup,data)

%This function handles switching between the various continuous dynamics
%phases.

code = data.exit;

setup.IC.th = data.state.th(end);
setup.IC.x = data.state.x(end);
setup.IC.y = data.state.y(end);
setup.IC.dth = data.state.dth(end);
setup.IC.dx = data.state.dx(end);
setup.IC.dy = data.state.dy(end);

setup.Tspan(1) = data.time(end);

if strcmp(code,'TIMEOUT')
    nextPhase = 'ABORT';
    setup = [];
    return;
end

switch phase
    case 'HINGE'
        switch code
            case 'SLIP_POS'
                nextPhase = 'SLIDE_POS';
            case 'SLIP_NEG'
                nextPhase = 'SLIDE_NEG';
            case 'LIFT'
                nextPhase = 'FLIGHT';
            case {'FALL_POS','FALL_NEG'}
                nextPhase = 'ABORT';
                setup = [];
            otherwise
                error('Invalid code in FSM')
        end
    case 'FLIGHT'
        switch code   % NEED TO WRITE IMPACT EQUATIONS!
            case 'STRIKE_0'
                nextPhase = 'ABORT';
                setup = [];
            case 'STRIKE_2'
                nextPhase = 'ABORT';   
                setup = [];
            otherwise
                error('Invalid code in FSM')
        end
    case 'SLIDE_POS'
        switch code
            case 'STUCK'
                nextPhase = 'HINGE';
            case 'LIFT'
                nextPhase = 'FLIGHT';
            case {'FALL_POS', 'FALL_NEG'}
                nextPhase = 'ABORT';
                setup = [];
            otherwise
                error('Invalid code in FSM')
        end
    case 'SLIDE_NEG'
        switch code
            case 'STUCK'
                nextPhase = 'HINGE';
            case 'LIFT'
                nextPhase = 'FLIGHT';
            case {'FALL_POS', 'FALL_NEG'}
                nextPhase = 'ABORT';
                setup = [];
            otherwise
                error('Invalid code in FSM')
        end
    otherwise
        error('Unrecognized phase')
end

end