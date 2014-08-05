function StateAfter = phaseMap(StateBefore,PhaseBefore,PhaseAfter)
% function StateAfter = phaseMap(StateBefore,PhaseBefore,PhaseAfter)
%
% Computer Generated File -- DO NOT EDIT
%
% This function was created by the function Write_PhaseMap()
% 10-Dec-2013 19:40:47
%
% Dymanics Model: retractable double pendulum biped
% Phases: Flight (F) == Both feet in the air
%         Double Stance (D) == Both feet on the ground
%         Single Stance One (S1) == Foot One -> ground, Foot Two -> air
%         Single Stance Two (S2) == Foot One -> air, Foot Two -> ground
%
% FUNCTION:
%   This function will compute the impact map that occurs during
%   transitions between any two phases of motion. Detailed assumptions can
%   be found in Derive_EoM.m
%
% ARGUMENTS:
%   StateBefore = [Nx1] state vector
%   PhaseBefore = element of set {'F','D','S1','S2'} = phase before impact
%   PhaseAfter = element of set {'F','D','S1','S2'} = phase after impact
%
% RETURNS:
%   StateAfter = [Nx1] state vector
%
% Matthew Kelly
% Cornell University
%
%
switch PhaseBefore    
    case 'F'    
        switch PhaseAfter    
            case 'F'    
                StateAfter = StateBefore;    
            case 'D' %Assume that collision happens on Foot One then Two    
                StateInt = MapOne(StateBefore);    
                StateAfter = MapTwo(StateInt);    
            case 'S1'    
                StateAfter = MapOne(StateBefore);    
            case 'S2'    
                StateAfter = MapTwo(StateBefore);    
            otherwise    
                error('Invalid phase after collision. Must be element of set: {''F'',''D'',''S1'',''S2''}');    
        end    
    case 'D'    
         StateAfter = StateBefore;    
    case 'S1'    
        switch PhaseAfter    
            case {'F','S1'}    
                StateAfter = StateBefore;    
            case {'D','S2'}    
                StateAfter = MapTwo(StateBefore);    
            otherwise    
                error('Invalid phase after collision. Must be element of set: {''F'',''D'',''S1'',''S2''}');    
        end    
    case 'S2'    
        switch PhaseAfter    
            case {'F','S2'}    
                StateAfter = StateBefore;    
            case {'D','S1'}    
                StateAfter = MapOne(StateBefore);    
            otherwise    
                error('Invalid phase after collision. Must be element of set: {''F'',''D'',''S1'',''S2''}');    
        end    
    otherwise    
                error('Invalid phase before collision. Must be element of set: {''F'',''D'',''S1'',''S2''}');    
end    

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  ~~~~~~~~~~~~~~~~~~~ Sub Functions  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function StateAfter = MapOne(StateBefore)    

x = StateBefore(1); % (m) Foot One horizontal position
y = StateBefore(2); % (m) Foot One vertical position
th1 = StateBefore(3); % (rad) Leg One absolute angle
th2 = StateBefore(4); % (rad) Leg Two absolute angle
L1 = StateBefore(5); % (m) Leg One length
L2 = StateBefore(6); % (m) Leg Two length
dx = StateBefore(7); % (m/s) Foot One horizontal velocity
dy = StateBefore(8); % (m/s) Foot One vertical velocity
dth1 = StateBefore(9); % (rad/s) Leg One absolute angular rate
dth2 = StateBefore(10); % (rad/s) Leg Two absolute angular rate
dL1 = StateBefore(11); % (m/s) Leg One extension rate
dL2 = StateBefore(12); % (m/s) Leg Two extensioin rate

StateAfter = zeros(size(StateBefore));
StateAfter(1:6,:) = StateBefore(1:6,:); % Configuration not changed by collision
StateAfter(7) = 0;
StateAfter(8) = 0;
StateAfter(9) = (dy*cos(th1) - dx*sin(th1) + L1*dth1*cos(th1)^2 + L1*dth1*sin(th1)^2)/(L1*(cos(th1)^2 + sin(th1)^2));
StateAfter(10) = dth2;
StateAfter(11) = (dL1*sin(th1)^2 + dx*cos(th1) + dy*sin(th1) + dL1*cos(th1)^2)/(cos(th1)^2 + sin(th1)^2);
StateAfter(12) = dL2;

end


function StateAfter = MapTwo(StateBefore)    

x = StateBefore(1); % (m) Foot One horizontal position
y = StateBefore(2); % (m) Foot One vertical position
th1 = StateBefore(3); % (rad) Leg One absolute angle
th2 = StateBefore(4); % (rad) Leg Two absolute angle
L1 = StateBefore(5); % (m) Leg One length
L2 = StateBefore(6); % (m) Leg Two length
dx = StateBefore(7); % (m/s) Foot One horizontal velocity
dy = StateBefore(8); % (m/s) Foot One vertical velocity
dth1 = StateBefore(9); % (rad/s) Leg One absolute angular rate
dth2 = StateBefore(10); % (rad/s) Leg Two absolute angular rate
dL1 = StateBefore(11); % (m/s) Leg One extension rate
dL2 = StateBefore(12); % (m/s) Leg Two extensioin rate

StateAfter = zeros(size(StateBefore));
StateAfter(1:6,:) = StateBefore(1:6,:); % Configuration not changed by collision
StateAfter(7) = dx;
StateAfter(8) = dy;
StateAfter(9) = dth1;
StateAfter(10) = -(dy*cos(th2) - dx*sin(th2) - dL1*cos(th1)*sin(th2) + dL1*cos(th2)*sin(th1) + L1*dth1*cos(th1)*cos(th2) + L1*dth1*sin(th1)*sin(th2))/(L2*(cos(th2)^2 + sin(th2)^2));
StateAfter(11) = dL1;
StateAfter(12) = -(dx*cos(th2) + dy*sin(th2) + dL1*cos(th1)*cos(th2) + dL1*sin(th1)*sin(th2) + L1*dth1*cos(th1)*sin(th2) - L1*dth1*cos(th2)*sin(th1))/(cos(th2)^2 + sin(th2)^2);

end
