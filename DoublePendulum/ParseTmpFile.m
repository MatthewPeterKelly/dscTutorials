function [CODE, Declarations, NewVars] = ParseTmpFile(FILENAME,Result_Name) 

%This function is designed to read the output of ccode.m back into matlab
%and parse it into a more useful format. It assumes a very specific format
%for the file, and assumes that the last line is a result, and
%automatically assigns it Result_Num.

%Figure out what to call things
persistent Result_Var_Num 
persistent Local_Var_Num    

%Initialize the persistent variables
if isempty(Result_Var_Num), Result_Var_Num=0; end
if isempty(Local_Var_Num), Local_Var_Num=2; end

%What should local variables be called?
BaseName = 'tmp';

%Check what the result name should be
if nargin == 1
    Result_Name = ['Result_' num2str(Result_Var_Num)];
    Result_Var_Num = Result_Var_Num + 1;  %Increment the number
end

% First, open the file and count the number of lines
fid = fopen(FILENAME);
N_Lines = 0;
while 1
    tline = fgetl(fid);
    if ~ischar(tline), break, end
    N_Lines = N_Lines + 1;
%     disp(tline)
end
% disp(N_Lines)
fclose(fid);

%Next, initialize the outputs;
CODE = cell(N_Lines,1);
Variables = cell(N_Lines,1);
Declarations = cell(2,1);
Declarations{1} = 'double  ';  %initialize the declarations for the tmp variables
Declarations{2} = 'double  ';  %initialize the declaration for the results

%Now, do the useful things:
fid = fopen(FILENAME);
for i=1:N_Lines
    tline = fgetl(fid);
    %Figure out what to name things:
    if i==N_Lines
        %This is the last line of the file -> it stores the result
        VarName = Result_Name;
        index = strfind(tline,'=');  %Search for the equal sign
        CODE{i} = [VarName ' = ' tline((index+1):end)];
        Declarations{2} = [Declarations{2} VarName];   %declare the result
    else
        %This is a regular line, so get the name from the file
        index = strfind(tline,'=');  %Search for the equal sign
        VarName = tline(1:(index-1)); %Get the first part of the string
        VarName = VarName(~isspace(VarName));  %Remove the spaces 
        CODE{i} = tline;
        if i==1
            %Then first line, don't need the comma
            Declarations{1} = [Declarations{1}  VarName ];  
        else
        Declarations{1} = [Declarations{1} ', ' VarName];   %Add it to the tmp pile
        end
    end
    
    Variables{i} = VarName;
end

Declarations{1} = [Declarations{1} ';'];
Declarations{2} = [Declarations{2} ';'];

fclose(fid);  %Close the file

%% Rename local variables
% This function will be called multiple times, and the output stored in a
% single file for all function calls. Thus, if the same variable names are
% used (such as t2,t3,t4...) then they will cause a compile error. To fix 
% this, the function uses a persistent variable to continue increasing the
% local variable ID numbers.

NewVars = Variables;   %Store the new variable names here
for i=1:(length(Variables)-1)
    NewVars{i} = [BaseName num2str(Local_Var_Num)];  %Create the new name
    Local_Var_Num = Local_Var_Num + 1;   %Increment the counter
end

%Now do a find and replace operation on all of the functions:
for i=1:length(CODE)
    for j = 1:length(NewVars)
        CODE{i} = strrep(CODE{i},Variables{j},NewVars{j});
    end
end

for i = 1:length(Declarations)
    for j = 1:length(NewVars)
        Declarations{i} = strrep(Declarations{i},Variables{j},NewVars{j});
    end
end

end