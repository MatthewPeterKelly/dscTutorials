function Write_Kinematics(FileWritingSetup)
% Write_Kinematics(FileWritingSetup)
%
% FUNCTION:
%
%   This function takes the symbolic expressions for system energy,
%   position, and velocity, and uses them to write a function file.
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
%   Parameters = [K x 2] cell array with all of the input parameters. The
%       first column is for names, the second for comments
%
%   Kinematics = A struct with fields for the symbolic expressions for
%       position and velocity.
%
%   Directory = the name of the directory to write the files in.
%
%
% OUTPUTS:
%   This function will write the function kinematics()
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
Kinematics = FileWritingSetup.Kinematics;
Parameters = FileWritingSetup.Parameters;
Directory = FileWritingSetup.Directory;
Energy = FileWritingSetup.Energy;

r0 = Kinematics.r0;
r1 = Kinematics.r1;
r2 = Kinematics.r2;
dr0 = Kinematics.dr0;
dr1 = Kinematics.dr1;
dr2 = Kinematics.dr2;

CurrDir = cd;
if ~exist(Directory,'dir')
    mkdir(Directory);
end
cd(Directory);


fid = fopen('kinematics.m','w');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Function Header                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fprintf(fid,['function [Position, Velocity, Energy] = ' ...
    'kinematics(States, Parameters)\n']);
fprintf(fid,['%% function [Position, Velocity, Energy] = ' ...
    'kinematics(States, Parameters)\n']);
fprintf(fid,'%%\n%% Computer Generated File -- DO NOT EDIT \n');
fprintf(fid,['%%\n%% This function was created by the function '...
    'Write_Kinematics()\n']);
fprintf(fid,['%% ' datestr(now) '\n%%\n']);
fprintf(fid,'%%  RETURNS: \n');
fprintf(fid,'%%   Energy = a struct with fields: \n');
fprintf(fid,'%%       Potential \n');
fprintf(fid,'%%       Kinetic \n');
fprintf(fid,'%%       Total \n');
fprintf(fid,'%% \n');
fprintf(fid,'%%   Position = a struct with fields: \n');
fprintf(fid,'%%       footOne = [2xN] matrix with the position vectors for Foot One \n');
fprintf(fid,'%%       footTwo = [2xN] matrix with the position vectors for Foot Two \n');
fprintf(fid,'%%       hip = [2xN] matrix with the position vectors for the hip \n');
fprintf(fid,'%%       CoM = [2xN] matrix with the position vectors for the center of mass\n');
fprintf(fid,'%% \n');
fprintf(fid,'%%   Velocity = a struct with fields: \n');
fprintf(fid,'%%       footOne = [2xN] matrix with the velocity vectors for Foot One \n');
fprintf(fid,'%%       footTwo = [2xN] matrix with the velocity vectors for Foot Two \n');
fprintf(fid,'%%       hip = [2xN] matrix with the velocity vectors for the hip \n');
fprintf(fid,'%%       CoM = [2xN] matrix with the velocity vectors for the center of mass \n');
fprintf(fid,'%%\n');
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

for idxParam=1:size(Parameters,1)
    fprintf(fid,[Parameters{idxParam,1} ' = Parameters.' ...
        Parameters{idxParam,1} '; %% ' ...
        Parameters{idxParam,2} '\n']);
end %idxState
fprintf(fid,'\n');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Write Outputs                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fprintf(fid,'N = size(States,2);\n');

fprintf(fid,'\n');
fprintf(fid,'Position.footOne = zeros(2,N);\n');
fprintf(fid,'Position.footTwo = zeros(2,N);\n');
fprintf(fid,'Position.hip = zeros(2,N);\n');
fprintf(fid,['Position.footOne(1,:) = ' vectorize(r0(1)) ';\n']);
fprintf(fid,['Position.footOne(2,:) = ' vectorize(r0(2)) ';\n']);
fprintf(fid,['Position.footTwo(1,:) = ' vectorize(r2(1)) ';\n']);
fprintf(fid,['Position.footTwo(2,:) = ' vectorize(r2(2)) ';\n']);
fprintf(fid,['Position.hip(1,:) = ' vectorize(r1(1)) ';\n']);
fprintf(fid,['Position.hip(2,:) = ' vectorize(r1(2)) ';\n']);
fprintf(fid,'Position.CoM = (m1*Position.footOne + m2*Position.footTwo + M*Position.hip)/(m1+m2+M);\n');
fprintf(fid,'\n');

fprintf(fid,'if nargout > 1');
fprintf(fid,'\n');
fprintf(fid,'    Velocity.footOne = zeros(2,N);\n');
fprintf(fid,'    Velocity.footTwo = zeros(2,N);\n');
fprintf(fid,'    Velocity.hip = zeros(2,N);\n');
fprintf(fid,['    Velocity.footOne(1,:) = ' vectorize(dr0(1)) ';\n']);
fprintf(fid,['    Velocity.footOne(2,:) = ' vectorize(dr0(2)) ';\n']);
fprintf(fid,['    Velocity.footTwo(1,:) = ' vectorize(dr2(1)) ';\n']);
fprintf(fid,['    Velocity.footTwo(2,:) = ' vectorize(dr2(2)) ';\n']);
fprintf(fid,['    Velocity.hip(1,:) = ' vectorize(dr1(1)) ';\n']);
fprintf(fid,['    Velocity.hip(2,:) = ' vectorize(dr1(2)) ';\n']);
fprintf(fid,'    Velocity.CoM = (m1*Velocity.footOne + m2*Velocity.footTwo + M*Velocity.hip)/(m1+m2+M);\n');
fprintf(fid,'\n');
fprintf(fid,'end\n');

fprintf(fid,'if nargout > 2');
fprintf(fid,'\n');
fprintf(fid,['    Energy.Potential.m1 = ' ...
    vectorize(Energy.Potential.m1) ';\n']);
fprintf(fid,['    Energy.Potential.m2 = ' ...
    vectorize(    Energy.Potential.m2) ';\n']);
fprintf(fid,['    Energy.Potential.M = ' ...
    vectorize(Energy.Potential.M) ';\n']);
fprintf(fid,'    Energy.Potential.Total = Energy.Potential.m1 + Energy.Potential.m2 + Energy.Potential.M;\n');
fprintf(fid,'\n');
fprintf(fid,['    Energy.Kinetic.m1 = ' ...
    vectorize(Energy.Kinetic.m1) ';\n']);
fprintf(fid,['    Energy.Kinetic.m2 = ' ...
    vectorize(Energy.Kinetic.m2) ';\n']);
fprintf(fid,['    Energy.Kinetic.M = ' ...
    vectorize(Energy.Kinetic.M) ';\n']);
fprintf(fid,'    Energy.Kinetic.Total = Energy.Kinetic.m1 + Energy.Kinetic.m2 + Energy.Kinetic.M;\n');
fprintf(fid,'\n');
fprintf(fid,'    Energy.Total = Energy.Kinetic.Total + Energy.Potential.Total;\n');
fprintf(fid,'\n');
fprintf(fid,'end\n');
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

