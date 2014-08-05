function Write_PhaseMap(FileWritingSetup)
% write_PhaseMap(FileWritingSetup)
%
% FUNCTION:
%
%   This function takes the symbolically derived impact map equations and
%   writes them to a function file.
%
%
% ARGUMENTS: FileWritingSetup.
%
%   SolnOne = A struct with a field containing an expression for the rates
%             immediately after a sticking collision with foot one
%
%   SolnTwo = A struct with a field containing an expression for the rates
%             immediately after a sticking collision with foot two
%
%   States = [2N x 2] cell array where the first column is the name of
%       the states, and the second column is comments about that states.
%       The index of a state in this array will match the index in the
%       dynamics function to be written. It is assumed that the second
%       block of N states are the time derivatives of the first block of N
%       states. Furthermore, it is assumed that the first N states are
%       positions and are not affected by impact.
%
%   Parameters = [K x 2] cell array with all of the input parameters. The
%       first column is for names, the second for comments
%
%   Directory = the name of the directory to write the files in.
%
%
% RETURNS:
%   This function will write a file for the dynamics equations in each
%   field of Dyn.
%
%
% NOTE:
%
%   1)  It is assumed that the symbols in all of the inputs are self
%       consistent; that is - if Contacts{1,1} = H1, then there must be a field
%       in Dyn.(FileNames{i,1}) called H1.
%
%
% Written by Matthew Kelly
% December 6, 2013
% Cornell University
%
% See also DERIVE_EOM

States = FileWritingSetup.States;
Directory = FileWritingSetup.Directory;

CurrDir = cd;
if ~exist(Directory,'dir')
    mkdir(Directory);
end
cd(Directory);

fid = fopen('phaseMap.m','w');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Function Header                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fprintf(fid,['function StateAfter = phaseMap(StateBefore,PhaseBefore,PhaseAfter)\n']);
fprintf(fid,'%% function StateAfter = phaseMap(StateBefore,PhaseBefore,PhaseAfter)\n');
fprintf(fid,'%%\n');
fprintf(fid,'%% Computer Generated File -- DO NOT EDIT\n');
fprintf(fid,'%%\n');
fprintf(fid,'%% This function was created by the function Write_PhaseMap()\n');
fprintf(fid,['%% ' datestr(now) '\n']);
fprintf(fid,'%%\n');
fprintf(fid,'%% Dymanics Model: retractable double pendulum biped\n');
fprintf(fid,'%% Phases: Flight (F) == Both feet in the air\n');
fprintf(fid,'%%         Double Stance (D) == Both feet on the ground\n');
fprintf(fid,'%%         Single Stance One (S1) == Foot One -> ground, Foot Two -> air\n');
fprintf(fid,'%%         Single Stance Two (S2) == Foot One -> air, Foot Two -> ground\n');
fprintf(fid,'%%\n');
fprintf(fid,'%% FUNCTION:\n');
fprintf(fid,'%%   This function will compute the impact map that occurs during\n');
fprintf(fid,'%%   transitions between any two phases of motion. Detailed assumptions can\n');
fprintf(fid,'%%   be found in Derive_EoM.m\n');
fprintf(fid,'%%\n');
fprintf(fid,'%% ARGUMENTS:\n');
fprintf(fid,'%%   StateBefore = [Nx1] state vector\n');
fprintf(fid,'%%   PhaseBefore = element of set {''F'',''D'',''S1'',''S2''} = phase before impact\n');
fprintf(fid,'%%   PhaseAfter = element of set {''F'',''D'',''S1'',''S2''} = phase after impact\n');
fprintf(fid,'%%\n');
fprintf(fid,'%% RETURNS:\n');
fprintf(fid,'%%   StateAfter = [Nx1] state vector\n');
fprintf(fid,'%%\n');
fprintf(fid,'%% Matthew Kelly\n');
fprintf(fid,'%% Cornell University\n');
fprintf(fid,'%%\n');
fprintf(fid,'%%\n');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Select Impact Map                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fprintf(fid,'switch PhaseBefore    \n');
fprintf(fid,'    case ''F''    \n');
fprintf(fid,'        switch PhaseAfter    \n');
fprintf(fid,'            case ''F''    \n');
fprintf(fid,'                StateAfter = StateBefore;    \n');
fprintf(fid,'            case ''D'' %%Assume that collision happens on Foot One then Two    \n');
fprintf(fid,'                StateInt = MapOne(StateBefore);    \n');
fprintf(fid,'                StateAfter = MapTwo(StateInt);    \n');
fprintf(fid,'            case ''S1''    \n');
fprintf(fid,'                StateAfter = MapOne(StateBefore);    \n');
fprintf(fid,'            case ''S2''    \n');
fprintf(fid,'                StateAfter = MapTwo(StateBefore);    \n');
fprintf(fid,'            otherwise    \n');
fprintf(fid,'                error(''Invalid phase after collision. Must be element of set: {''''F'''',''''D'''',''''S1'''',''''S2''''}'');    \n');
fprintf(fid,'        end    \n');
fprintf(fid,'    case ''D''    \n');
fprintf(fid,'         StateAfter = StateBefore;    \n');
fprintf(fid,'    case ''S1''    \n');
fprintf(fid,'        switch PhaseAfter    \n');
fprintf(fid,'            case {''F'',''S1''}    \n');
fprintf(fid,'                StateAfter = StateBefore;    \n');
fprintf(fid,'            case {''D'',''S2''}    \n');
fprintf(fid,'                StateAfter = MapTwo(StateBefore);    \n');
fprintf(fid,'            otherwise    \n');
fprintf(fid,'                error(''Invalid phase after collision. Must be element of set: {''''F'''',''''D'''',''''S1'''',''''S2''''}'');    \n');
fprintf(fid,'        end    \n');
fprintf(fid,'    case ''S2''    \n');
fprintf(fid,'        switch PhaseAfter    \n');
fprintf(fid,'            case {''F'',''S2''}    \n');
fprintf(fid,'                StateAfter = StateBefore;    \n');
fprintf(fid,'            case {''D'',''S1''}    \n');
fprintf(fid,'                StateAfter = MapOne(StateBefore);    \n');
fprintf(fid,'            otherwise    \n');
fprintf(fid,'                error(''Invalid phase after collision. Must be element of set: {''''F'''',''''D'''',''''S1'''',''''S2''''}'');    \n');
fprintf(fid,'        end    \n');
fprintf(fid,'    otherwise    \n');
fprintf(fid,'                error(''Invalid phase before collision. Must be element of set: {''''F'''',''''D'''',''''S1'''',''''S2''''}'');    \n');
fprintf(fid,'end    \n');
fprintf(fid,'\n');
fprintf(fid,'end\n');
fprintf(fid,'\n\n');
fprintf(fid,'%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n');
fprintf(fid,'%%%%%%%%  ~~~~~~~~~~~~~~~~~~~ Sub Functions  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%%%%%%\n');
fprintf(fid,'%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n');
fprintf(fid,'\n\n');




%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Write MapOne subfunction                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

    fprintf(fid,'function StateAfter = MapOne(StateBefore)    \n\n');

    fprintf(fid,'StateAfter = StateBefore;\n');
        
    for idxState=1:size(States,1)
        if (strcmp(States{idxState,1},'dx1') || strcmp(States{idxState,1},'dy1'))
            fprintf(fid, ['StateAfter(:,' num2str(idxState) ') = 0*StateAfter(:,' num2str(idxState) ')'...
                '; %% ' States{idxState,2} '\n']);
        end
    end %idxState
    fprintf(fid,'\n');

    fprintf(fid,'end\n\n\n');



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Write MapTwo subfunction                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


 
    fprintf(fid,'function StateAfter = MapTwo(StateBefore)    \n\n');

    fprintf(fid,'StateAfter = StateBefore;\n');
        
    for idxState=1:size(States,1)
        if (strcmp(States{idxState,1},'dx2') || strcmp(States{idxState,1},'dy2'))
            fprintf(fid, ['StateAfter(:,' num2str(idxState) ') = 0*StateAfter(:,' num2str(idxState) ')'...
                '; %% ' States{idxState,2} '\n']);
        end
    end %idxState
    fprintf(fid,'\n');

    fprintf(fid,'end\n\n\n');


fclose(fid);

cd(CurrDir);

end
