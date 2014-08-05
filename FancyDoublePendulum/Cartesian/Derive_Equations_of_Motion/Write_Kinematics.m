function Write_Kinematics(FileWritingSetup)
% Write_Power(FileWritingSetup)
%
% FUNCTION:
%
%   This function writes a function that computes the power being used by
%   the actuators on the robot.
%
%
% INPUTS: FileWritingSetup.
%
%   States = [2N x 2] cell array where the first column is the name of
%       the states, and the second column is comments about that states.
%       The index of a state in this array will match the index in the
%       dynamics function to be written. This function assumes a second
%       order system that is being written in first order form, so if
%       States{i,1} = z, then States{i+N,1} = dz; Furthermore, the output
%       (dStates) will have the form: dStates(i,:) = dz,
%       and dStates(i+N,:) = ddz.
%
%   Kinematics = a struct with polar coordinate kinematics
%
%   Directory = the name of the directory to write the files in.
%
%
% OUTPUTS:
%   This function will write the function power()
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
% December 8, 2013
% Cornell University
%
% See also DERIVE_EOM

States = FileWritingSetup.States;
Directory = FileWritingSetup.Directory;
Kinematics = FileWritingSetup.Kinematics;

CurrDir = cd;
if ~exist(Directory,'dir')
    mkdir(Directory);
end
cd(Directory);


fid = fopen('kinematics.m','w');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Function Header                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fprintf(fid,'function Kinematics = kinematics(States)\n');
fprintf(fid,'%% function Kinematics = kinematics(States)\n');
fprintf(fid,'%%\n%% Computer Generated File -- DO NOT EDIT \n');
fprintf(fid,['%%\n%% This function was created by the function '...
    'Write_Power()\n']);
fprintf(fid,['%% ' datestr(now) '\n%%\n']);
fprintf(fid,'%%  ARGUMENTS: \n');
fprintf(fid,'%%   States = [Nstate x Ntime] matrix of states \n');
fprintf(fid,'%% \n');
fprintf(fid,'%%  RETURNS: \n');
fprintf(fid,'%%   Kinematics = a struct with fields:   \n');
fprintf(fid,'%%      th1 = angle of leg 1   \n');
fprintf(fid,'%%      th2 = angle of leg 2   \n');
fprintf(fid,'%%      L1 = length of leg 1   \n');
fprintf(fid,'%%      L2 = length of leg 2   \n');
fprintf(fid,'%%      dth1 = (d/dt) angle of leg 1   \n');
fprintf(fid,'%%      dth2 = (d/dt) angle of leg 2   \n');
fprintf(fid,'%%      dL1 = (d/dt) length of leg 1   \n');
fprintf(fid,'%%      dL2 = (d/dt) length of leg 2   \n');
fprintf(fid,'%% \n');
fprintf(fid,'%%\n');
fprintf(fid,'%% Matthew Kelly \n%% Cornell University \n%% \n\n');
fprintf(fid,'%% See also DERIVE_EOM \n');
fprintf(fid,'%% \n');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Parse Inputs                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

for idxState=1:size(States,1)
    fprintf(fid,[States{idxState,1} ' = States(:,' num2str(idxState) '); %% ' ...
        States{idxState,2} '\n']);
end %idxState
fprintf(fid,'\n');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%         Intermediate Steps and Common Expressions                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fprintf(fid,'%% Commonly used expressions\n');
fNames = fieldnames(Kinematics);
for i=1:length(fNames)
    if strcmp(fNames{i},'L1') || strcmp(fNames{i},'L2')  %Need these ones!
        fprintf(fid,[fNames{i} ' = ' vectorize(Kinematics.(fNames{i})) ';\n']);  
    else  %Don't need them - comment out
        fprintf(fid,['%% ' fNames{i} ' = ' char(Kinematics.(fNames{i})) ';\n']);  
    end
end
fprintf(fid,'\n');


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Compute Kinematics                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fNames = fieldnames(Kinematics);
for i=1:length(fNames)
    if strcmp(fNames{i},'L1')
        fprintf(fid,'Kinematics.L1 = L1;\n');
    elseif strcmp(fNames{i},'L2')
        fprintf(fid,'Kinematics.L2 = L2;\n');
    else
    fprintf(fid,['Kinematics.' fNames{i} ' = ' vectorize(Kinematics.(fNames{i})) ';\n']);
end
end
fprintf(fid,'\nend\n');

cd(CurrDir);

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%  ~~~~~~~~~~~~~~~~~~~ Sub Functions  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function characterExpression = vectorize(symbolicExpression)

        characterExpression = char(symbolicExpression);
        characterExpression = strrep(characterExpression,'*','.*');
        characterExpression = strrep(characterExpression,'/','./');
        characterExpression = strrep(characterExpression,'^','.^');
end

