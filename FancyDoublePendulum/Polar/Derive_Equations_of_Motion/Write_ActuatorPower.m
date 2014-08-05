function Write_ActuatorPower(FileWritingSetup)
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
%   Power = struct with a field for each actuator.
%
%   Actuators = [P x 2] cell array giving the names of each of the
%       actuators and their comments.
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
Power = FileWritingSetup.Power;
Directory = FileWritingSetup.Directory;
Actuators = FileWritingSetup.Actuators;

CurrDir = cd;
if ~exist(Directory,'dir')
    mkdir(Directory);
end
cd(Directory);


fid = fopen('actuatorPower.m','w');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Function Header                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fprintf(fid,'function Power = actuatorPower(States, Actuators, Phase)\n');
fprintf(fid,'%% function Power = actuatorPower(States, Actuators, Phase)\n');
fprintf(fid,'%%\n%% Computer Generated File -- DO NOT EDIT \n');
fprintf(fid,['%%\n%% This function was created by the function '...
    'Write_Power()\n']);
fprintf(fid,['%% ' datestr(now) '\n%%\n']);
fprintf(fid,'%%  ARGUMENTS: \n');
fprintf(fid,'%%   States = [Nstate x Ntime] matrix of states \n');
fprintf(fid,'%%   Actuator = [Nactuator x Ntime] matrix of actuator values \n');
fprintf(fid,'%%   Phase = {''D'',''F'',''S1'',S2''} or omit. \n');
fprintf(fid,'%%       Since the ankle torques are acting against the ground, they should \n');
fprintf(fid,'%%       be set to zero when a given leg is in flight. If Phase is included, \n');
fprintf(fid,'%%       then this function will force these inputs to be zero. If it is \n');
fprintf(fid,'%%       ommitted, then it is assumed that the user has done that error \n');
fprintf(fid,'%%       checking. \n');
fprintf(fid,'%% \n');
fprintf(fid,'%%  RETURNS: \n');
fprintf(fid,'%%   Power = a struct with fields for each actuator: \n');
fprintf(fid,'%%       leg one (linear force actuator) \n');
fprintf(fid,'%%       leg two (linear force actuator) \n');
fprintf(fid,'%%       ankle one (torque actuator) \n');
fprintf(fid,'%%       ankle two (torque actuator) \n');
fprintf(fid,'%%       hip (torque from leg one on leg two \n');
fprintf(fid,'%% \n');
fprintf(fid,'%%\n');
fprintf(fid,'%% Matthew Kelly \n%% Cornell University \n%% \n\n');
fprintf(fid,'%% \n');
fprintf(fid,'%% See also DERIVE_EOM \n');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Parse Inputs                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

for idxState=1:size(States,1)
    fprintf(fid,[States{idxState,1} ' = States(' num2str(idxState) ',:); %% ' ...
        States{idxState,2} '\n']);
end %idxState
fprintf(fid,'\n');

fprintf(fid,'if nargin==2\n');
fprintf(fid,'    Phase = '''';\n');
fprintf(fid,'end\n');
fprintf(fid,' \n');

for idxAct=1:size(Actuators,1)    
    
    if strcmp(Actuators{idxAct,1},'T1')
        fprintf(fid,'if strcmp(Phase,''F'') || strcmp(Phase, ''S2'') \n');
        fprintf(fid,['    ' Actuators{idxAct,1} ' = zeros(1,size(Actuators,2)); %%Foot One not in contact with ground!\n']);
        fprintf(fid,'else\n');
        fprintf(fid,['    ' Actuators{idxAct,1} ' = Actuators(' num2str(idxAct) ',:); %% ' Actuators{idxAct,2} '\n']);
        fprintf(fid,'end\n');
    elseif strcmp(Actuators{idxAct,1},'T2')
        fprintf(fid,'if strcmp(Phase,''F'') || strcmp(Phase, ''S1'') \n');
        fprintf(fid,['    ' Actuators{idxAct,1} ' = zeros(1,size(Actuators,2)); %%Foot Two not in contact with ground!\n']);
        fprintf(fid,'else\n');
        fprintf(fid,['    ' Actuators{idxAct,1} ' = Actuators(' num2str(idxAct) ',:); %% ' Actuators{idxAct,2} '\n']);
        fprintf(fid,'end\n');      
    else
        fprintf(fid,[Actuators{idxAct,1} ' = Actuators(' num2str(idxAct) ',:); %% ' Actuators{idxAct,2} '\n']);
    end
   
end %idxAct
fprintf(fid,'\n');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Write Outputs                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

FieldNames = fieldnames(Power);
for i=1:length(FieldNames)
    fprintf(fid,['Power.' FieldNames{i} ' = ' vectorize(Power.(FieldNames{i})) ';\n']);
end
fprintf(fid,'\n');
fprintf(fid,'end\n');
fclose(fid);

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

