function plotInfo = getPlotInfo(output)

%This function formats the output from gpops2 and does some additional
%calculations for kinematics, energy, power, ect.

% For now, use matrix convention: [Ndata, Ntime]

nPhase = length(output.result.setup.auxdata.phase);
plotInfo.data = zeros(nPhase,0);
plotInfo.parameters = output.result.setup.auxdata;

for iphase=1:nPhase;
    
    %Store the code for the current phase of motion.
    try
        plotInfo.data(iphase).phase(iphase) = plotInfo.parameters.phase(iphase);
    catch ME
       disp('--> Make sure that auxdata.phase(iphase) exists');
       disp('--> This should contain a value in the set {''D'',''S1'',''S2'',''F''}')
       throw(ME);
    end
    
    %Copy the existing solution over to the output
      plotInfo.data(iphase).time = output.result.solution.phase(iphase).time';
      plotInfo.data(iphase).state = output.result.solution.phase(iphase).state';
      plotInfo.data(iphase).control = output.result.solution.phase(iphase).control';
      plotInfo.data(iphase).integral = output.result.solution.phase(iphase).integral';
         
    %Get kinematics and energy info:
    States = plotInfo.data(iphase).state;
    Parameters = output.result.setup.auxdata.dynamics;
    [Position, Velocity, Energy] = kinematics(States, Parameters);
    plotInfo.data(iphase).position = Position;
    plotInfo.data(iphase).velocity = Velocity;
    plotInfo.data(iphase).energy = Energy;
    
    %Get the power used by the actuators:
    Actuators = plotInfo.data(iphase).control;
    Phase = plotInfo.data(iphase).phase(iphase);
    plotInfo.data(iphase).power = actuatorPower(States, Actuators, Phase);
    
end






end