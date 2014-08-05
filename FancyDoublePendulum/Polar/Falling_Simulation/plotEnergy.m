function plotEnergy(SimDat,P)

%This function generates a plot showing the system energy

nPoints = 1000;
Duration = SimDat{end}.sol.x(end) - SimDat{1}.sol.x(1);
ptsPerSec = nPoints/Duration;

%Build a single data set for all phases
StateData = [];
TimeData = [];
PowerData.legOne = [];
PowerData.legTwo = [];
PowerData.ankleOne = [];
PowerData.ankleTwo = [];
PowerData.hip = [];
actNames = fieldnames(PowerData);
for idxPh=1:length(SimDat)
   Tspan = [SimDat{idxPh}.sol.x(1), SimDat{idxPh}.sol.x(end)];
    N = ceil(ptsPerSec*diff(Tspan));
    Time = linspace(Tspan(1),Tspan(2),N);
    State = deval(SimDat{idxPh}.sol,Time);
    TimeData = [TimeData,Time];
    StateData = [StateData,State];
    
    Actuators = Controller_PD(State,P.Control, SimDat{idxPh}.phase);
    Power = actuatorPower(State, Actuators, SimDat{idxPh}.phase);
    for idxAct=1:length(actNames)
        PowerData.(actNames{idxAct}) = [PowerData.(actNames{idxAct}), Power.(actNames{idxAct})];
    end
end

[~, ~, Energy] = kinematics(StateData, P.Dyn);

%If the gains are all zero except for the P gain on leg length, then the
%system behaves exactly the same as a spring double pendulum, which
%conserves energy. Let's Test:
GainCheck = false(8,1);
GainCheck(1) = P.Control.footOne.Kp == 0;
GainCheck(2) = P.Control.footOne.Kd == 0;
GainCheck(3) = P.Control.footTwo.Kp == 0;
GainCheck(4) = P.Control.footTwo.Kd == 0;
GainCheck(5) = P.Control.hip.Kp == 0;
GainCheck(6) = P.Control.hip.Kd == 0;
GainCheck(7) = P.Control.legOne.Kd == 0;
GainCheck(8) = P.Control.legTwo.Kd == 0;
ConservesEnergy = sum(GainCheck)==8;


LineColor_One = 'g';
LineColor_Two = 'r';
LineColor_Hip = 'k';
if ConservesEnergy %Then spring pendulum mode -> energy conserved

    %Convert to a named struct
    S = convert(StateData);   

    %Compute energy stored in the actuator of leg one
    C = P.Control.legOne;
    pos = S.L1;    
    Energy.Spring.One = 0.5*C.Kp*(C.nomPos-pos).^2;

    %Compute energy stored in the actuator of leg two
    C = P.Control.legTwo;
    pos = S.L2;    
    Energy.Spring.Two = 0.5*C.Kp*(C.nomPos-pos).^2;
    
    %Combine into the energy struct
    Energy.Spring.Total = Energy.Spring.One + Energy.Spring.Two;
    Energy.Total = Energy.Total + Energy.Spring.Total;
    
    %Plots!
    subplot(4,1,1); hold on;
        plot(TimeData,Energy.Kinetic.m1, LineColor_One)
        plot(TimeData,Energy.Kinetic.m2, LineColor_Two)
        plot(TimeData,Energy.Kinetic.M, LineColor_Hip)
        showPhase(SimDat)
        ylabel('Energy (J)')
        title('Kinetic Energy')
        legend('Foot 1','Foot 2','Hip')
    subplot(4,1,2); hold on;
        plot(TimeData,Energy.Potential.m1, LineColor_One)
        plot(TimeData,Energy.Potential.m2, LineColor_Two)
        plot(TimeData,Energy.Potential.M, LineColor_Hip)
        showPhase(SimDat)
        ylabel('Energy (J)')
        title('Gravitational Potential Energy')
        legend('Foot 1','Foot 2','Hip')
  	subplot(4,1,3); hold on;
        plot(TimeData,Energy.Spring.One,LineColor_One)
        plot(TimeData,Energy.Spring.Two,LineColor_Two)
        legend('One','Two')
        showPhase(SimDat)
        ylabel('Energy (J)')
         title('Spring Potential Energy')
    subplot(4,1,4); hold on;
        plot(TimeData,Energy.Total,'b-')
        showPhase(SimDat)
        ylabel('Energy (J)')
         title('Total Mechanical Energy  --  This should be conserved within each phase')
         
else
  %Plots!
    subplot(4,1,1); hold on;
        plot(TimeData,Energy.Kinetic.m1, LineColor_One)
        plot(TimeData,Energy.Kinetic.m2, LineColor_Two)
        plot(TimeData,Energy.Kinetic.M, LineColor_Hip)
        showPhase(SimDat)
        ylabel('Energy (J)')
        title('Kinetic Energy')
        legend('Foot 1','Foot 2','Hip')
    subplot(4,1,2); hold on;
        plot(TimeData,Energy.Potential.m1, LineColor_One)
        plot(TimeData,Energy.Potential.m2, LineColor_Two)
        plot(TimeData,Energy.Potential.M, LineColor_Hip)
        showPhase(SimDat)
        ylabel('Energy (J)')
        title('Gravitational Potential Energy')
        legend('Foot 1','Foot 2','Hip')
    subplot(4,1,3); hold on;
        plot(TimeData,Energy.Total)
        showPhase(SimDat)
        ylabel('Energy (J)') 
        title('Total Mechanical Energy')
    subplot(4,1,4); hold on;
        plot(TimeData,PowerData.ankleOne, [LineColor_One ':'])
        plot(TimeData,PowerData.ankleTwo, [LineColor_Two ':'])
        plot(TimeData,PowerData.legOne, LineColor_One)
        plot(TimeData,PowerData.legTwo, LineColor_Two)
        plot(TimeData,PowerData.hip, LineColor_Hip)
        showPhase(SimDat)
        ylabel('Power (W)')
        title('Actuator Power Use')
     	legend('ankleOne', 'ankleTwo', 'legOne','legTwo','hip')
end

end

function showPhase(SimDat)

extents = axis;
Y = extents(3:4);

time = SimDat{1}.sol.x(1);
plot(time*[1,1],Y,'k:')

for idxPh=1:length(SimDat)
    Tspan = [ SimDat{idxPh}.sol.x(1), SimDat{idxPh}.sol.x(end)];
    text(mean(Tspan),mean(Y),SimDat{idxPh}.phase);
    plot(Tspan(2)*[1,1],Y,'k:');
end

end