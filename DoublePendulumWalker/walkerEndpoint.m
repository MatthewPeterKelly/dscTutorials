function output = walkerEndpoint(input)

auxdata = input.auxdata;
P = auxdata.dynamics;
PushOffImpluse = input.parameter(1);
idx = auxdata.index;


%-------------------------------------------------------------------------%
%-------------------------- Constraints ----------------------------------%
%-------------------------------------------------------------------------%

Xm = input.phase(1).finalstate;
J = PushOffImpluse;
[Xp, Step_Vector, Slope] = Double_Pendulum_HeelStrike(Xm',J,P);

periodicDefect = Xp' - input.phase(1).initialstate;

duration = input.phase(1).finaltime - input.phase(1).initialtime;

output.eventgroup(idx.cst.PERIODIC).event = periodicDefect;
output.eventgroup(idx.cst.SLOPE).event = Slope;
output.eventgroup(idx.cst.STEP_LENGTH).event = norm(Step_Vector);
output.eventgroup(idx.cst.VELOCITY).event = Step_Vector(1)/duration;

%-------------------------------------------------------------------------%
%---------------------- Cost Function ------------------------------------%
%-------------------------------------------------------------------------%

switch auxdata.cost.model
    case 'TorqueSquared' %Time Integral of Torque^2 
        k = auxdata.cost.ImpulseConstant;
        PushOffCost = k*PushOffImpluse^2;
        output.objective = PushOffCost + sum(input.phase(1).integral);

    case 'CostOfTransport' %Time Integral of Torque*Rate
        Total_Energy = PushOffImpluse + sum(input.phase(1).integral);
            Weight = P.g*(P.m1+P.m2);
            Distance = norm(Step_Vector);
            output.objective = Total_Energy/(Weight*Distance);
    otherwise
        error('Invalid Cost Method');
end

end