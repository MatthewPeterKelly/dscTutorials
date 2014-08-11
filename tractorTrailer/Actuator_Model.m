function Input = Actuator_Model(Command, Params)

%This function is used to saturate the actuators

persistent Cmd_Prev

if isempty(Cmd_Prev)
    Cmd_Prev = Command;
end

%Actuator Rate Saturation
dt = Params.Ctl.dt;   %Set in Simulator 
Rate = (Command - Cmd_Prev)/dt;
Low = Rate < Params.Ctl.Rate_Limits(:,1);
Upp = Rate > Params.Ctl.Rate_Limits(:,2);

Command(Low) = Cmd_Prev(Low) + Params.Ctl.Rate_Limits(Low,1)*dt;
Command(Upp) = Cmd_Prev(Upp) + Params.Ctl.Rate_Limits(Upp,2)*dt;

% Actuator Saturation
Low = Command < Params.Ctl.Actuator_Limits(:,1);
Upp = Command > Params.Ctl.Actuator_Limits(:,2);

Command(Low) = Params.Ctl.Actuator_Limits(Low,1);
Command(Upp) = Params.Ctl.Actuator_Limits(Upp,2);

Input = Command;   %Store the results to return

%Store persistent variables for the next function call
Cmd_Prev = Command;

end