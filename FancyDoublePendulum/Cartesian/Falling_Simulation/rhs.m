function dStates = rhs(~,States,Phase,P)

%This function is a wrapper for the controller and dynamics functions that
%makes them easier to call from ode45.


Actuators = Controller_PD(States(:)',P.Control,Phase);

switch Phase
    case 'F'
        dStates = dynamics_flight(States(:)',Actuators,P.Dyn)';
    case 'D'
        dStates = dynamics_doubleStance(States(:)',Actuators,P.Dyn)';
    case 'S1'
        dStates = dynamics_singleStanceOne(States(:)',Actuators,P.Dyn)';
    case 'S2'
        dStates = dynamics_singleStanceTwo(States(:)',Actuators,P.Dyn)';
    otherwise
        error('Invalid phase, must be element of {''F'',''D'',''S1'',''S2''}');
end

end