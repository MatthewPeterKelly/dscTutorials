function StateAfter = phaseMap(StateBefore,PhaseBefore,PhaseAfter)
% function StateAfter = phaseMap(StateBefore,PhaseBefore,PhaseAfter)
%
% Computer Generated File -- DO NOT EDIT
%
% This function was created by the function Write_PhaseMap()
% 13-Dec-2013 11:41:00
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

StateAfter = StateBefore;
StateAfter(:,9) = 0*StateAfter(:,9); % (m/s) Foot One horizontal velocity
StateAfter(:,10) = 0*StateAfter(:,10); % (m/s) Foot One vertical velocity

end


function StateAfter = MapTwo(StateBefore)    

StateAfter = StateBefore;
StateAfter(:,11) = 0*StateAfter(:,11); % (m/s) Foot Two horizontal velocity
StateAfter(:,12) = 0*StateAfter(:,12); % (m/s) Foot Two vertical velocity

end


