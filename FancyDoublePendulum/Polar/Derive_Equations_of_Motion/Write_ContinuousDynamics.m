function Write_ContinuousDynamics(FileWritingSetup)
% Write_ContinuousDynamics(FileWritingSetup)
%
% FUNCTION:
%
%   This function takes the symbolically derived equations of motion for
%   the retractable double pendulum biped model and writes them to function
%   files.
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
%   Actuators = [P x 2] cell array giving the names of each of the
%       actuators and their comments.
%
%   Contacts = [M x 2] cell array giving all of the contact forces to
%       output. The first column is names, second column is comments
%
%   Parameters = [K x 2] cell array with all of the input parameters. The
%       first column is for names, the second for comments
%
%   FileData = [L x 3+?] cell array with the names of each field in Dyn in
%       the first column, and the file name to write those dynamics
%       equations to. The final column is used for comments that are
%       specific to those dynamics. Seperate multiple lines by commas:
%       'comment line one', 'comment line two', ...
%
%   Dyn = A struct, with a field for each dynamics equation to write. Every
%       field must contain a symbolic expression for each of the state
%       accelerations and contact forces.
%
%   Directory = the name of the directory to write the files in.
%
%
% OUTPUTS:
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
Dyn = FileWritingSetup.Dyn;
Contacts = FileWritingSetup.Contacts;
Actuators = FileWritingSetup.Actuators;
Parameters = FileWritingSetup.Parameters;
FileData = FileWritingSetup.FileData;
Directory = FileWritingSetup.Directory;

CurrDir = cd;
if ~exist(Directory,'dir')
    mkdir(Directory);
end
cd(Directory);

for idxPhase = 1:size(FileData,1)
    fid = fopen([FileData{idxPhase,2} '.m'],'w');
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                         Function Header                             %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    
    fprintf(fid,['function [dStates, contactForces] = ' ...
        FileData{idxPhase,2} '(States, Actuators, Parameters)\n']);
    fprintf(fid,['%% function [dStates, Actuators] = ' ...
        FileData{idxPhase,2} '(States, Actuators, Parameters)\n']);
    fprintf(fid,'%%\n%% Computer Generated File -- DO NOT EDIT \n');
    fprintf(fid,['%%\n%% This function was created by the function '...
        'Write_ContinuousDynamics()\n']);
    fprintf(fid,['%% ' datestr(now) '\n%%\n']);
    for idxComment = 3:length(FileData(idxPhase,:))
        fprintf(fid,['%% ' FileData{idxPhase,idxComment} '\n']);
    end %idxComment
    fprintf(fid,'%%\n');
    fprintf(fid,'%% Matthew Kelly \n%% Cornell University \n%% \n\n');
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                           Parse Inputs                              %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    
    for idxState=1:size(States,1)
        fprintf(fid,[States{idxState,1} ' = States(' num2str(idxState) ',:); %% ' ...
            States{idxState,2} '\n']);
    end %idxState
    fprintf(fid,'\n');
    
    for idxAct=1:size(Actuators,1)
        rhs = [' = Actuators(' num2str(idxAct) ',:); %% ' Actuators{idxAct,2} '\n'];
        %Need to make sure that we disconnect the ankle motors when their
        %respective feed are not in contact with the ground!
        if strcmp(FileData{idxPhase,1},'Flight') || strcmp(FileData{idxPhase,1},'SingleTwo')
            if strcmp(Actuators{idxAct,1},'T1')
                rhs = ' = 0; %%Foot One not in contact with ground!\n';
            end
        end
        if strcmp(FileData{idxPhase,1},'Flight') || strcmp(FileData{idxPhase,1},'SingleOne')
            if strcmp(Actuators{idxAct,1},'T2')
                rhs = ' = 0; %%Foot Two not in contact with ground!\n';
            end
        end
        fprintf(fid,[Actuators{idxAct,1} rhs]);
    end %idxAct
    fprintf(fid,'\n');
    
    for idxParam=1:size(Parameters,1)
        fprintf(fid,[Parameters{idxParam,1} ' = Parameters.' ...
            Parameters{idxParam,1} '; %% ' ...
            Parameters{idxParam,2} '\n']);
    end %idxState
    fprintf(fid,'\n');
   
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                   Explicitly write constraints                      %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    
    Ndof = size(States,1)/2;
    fprintf(fid,'%% Constraints for this phase: \n');
    for i=1:Ndof
        writeIndex = i + Ndof;
        writeName = ['d' States{writeIndex,1}];
        writeData = Dyn.(FileData{idxPhase,1}).(writeName);
        if (writeData==0)
            fprintf(fid,[writeName ' = 0; %%' States{i,2} '\n']);
        end
    end
    for i=1:size(Contacts,1)
        writeName = Contacts{i,1};
        writeData = Dyn.(FileData{idxPhase,1}).(writeName);
        if (writeData==0)
            fprintf(fid,[writeName ' = 0; %%' Contacts{i,2} '\n']);
        end
    end
    fprintf(fid,'\n');
      
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                       Write Accelerations                           %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    
     N = num2str(Ndof);
    fprintf(fid,'dStates = zeros(size(States));\n');
    fprintf(fid,['dStates(1:' N ',:) = '...
        'States((1+' N '):(' N '+' N '),:);\n']);
    for idxAcc=1:Ndof
        writeIndex = idxAcc + Ndof;
        writeName = ['d' States{writeIndex,1}];
        writeData = Dyn.(FileData{idxPhase,1}).(writeName);
        vectorizedExpression = vectorize(writeData);
        fprintf(fid,['dStates(' num2str(writeIndex) ',:) = ' ...
            vectorizedExpression ';\n']);
    end %idxAcc
    fprintf(fid,'\n');
  
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                       Write Contact Forces                          %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

    for idxContact=1:size(Contacts,1)
        fprintf(fid,['%% contactForces(' num2str(idxContact) ',:) == '...
            Contacts{idxContact,1} ' == ' Contacts{idxContact,2} '\n']);
    end %idxContact
    fprintf(fid,['contactForces = zeros(' num2str(size(Contacts,1))...
        ',size(States,2));\n']);
    for idxContact=1:size(Contacts,1)
        writeIndex = num2str(idxContact);
        writeName = Contacts{idxContact,1};
        writeData = Dyn.(FileData{idxPhase,1}).(writeName);
        vectorizedExpression = vectorize(writeData);
        fprintf(fid,['contactForces(' writeIndex ',:) = ' ...
            vectorizedExpression ';\n']);
    end %idxContact
    fprintf(fid,'\n');
    
    fprintf(fid,'end\n');
    fclose(fid);
end %idxPhase
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
characterExpression = strrep(characterExpression,...
    '0','zeros(1,size(States,2))');

end


