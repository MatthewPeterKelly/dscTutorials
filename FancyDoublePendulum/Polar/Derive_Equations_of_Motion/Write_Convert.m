function Write_Convert(FileWritingSetup)

Directory = FileWritingSetup.Directory;
FileWritingSetup = rmfield(FileWritingSetup,'Directory');

Fields = fieldnames(FileWritingSetup);
Lengths = zeros(length(Fields),1);
for i=1:length(Fields)
    Lengths(i) = size(FileWritingSetup.(Fields{i}),1);
end
if length(Lengths) ~= length(unique(Lengths))
    %Then it is no longer possible to identify which type of data based on
    %the number of data channels. Need to rewrite this function.
    error('Each type of DataIn must have a different number of channels')
end

CurrDir = cd;
if ~exist(Directory,'dir')
    mkdir(Directory);
end
cd(Directory);

fid = fopen('convert.m','w');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Function Header                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fprintf(fid,'function DataOut = convert(DataIn)\n');
fprintf(fid,'%% function DataOut = convert(DataIn)\n');
fprintf(fid,'%%\n');
fprintf(fid,'%% Computer Generated File -- DO NOT EDIT\n');
fprintf(fid,'%%\n');
fprintf(fid,'%% This function was created by the function Write_Convert()\n');
fprintf(fid,['%% ' datestr(now) '\n']);
fprintf(fid,'%%\n');
fprintf(fid,'%% This function will convert between special structs and matricies. \n');
fprintf(fid,'%% The input must match exactly with one of the cases below.\n');
fprintf(fid,'%%\n');
fprintf(fid,'%% Matthew Kelly\n');
fprintf(fid,'%% Cornell University\n');
fprintf(fid,'%%\n');
fprintf(fid,'%%\n');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Struct -> Matrix                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fprintf(fid,'if isstruct(DataIn)    \n');
fprintf(fid,'    %%Then converting from a struct to a matrix    \n');
fprintf(fid,'    Names = fieldnames(DataIn);    \n');
fprintf(fid,'    switch length(Names)    \n');

for idxField=1:length(Fields)
    name = Fields{idxField};
    data = FileWritingSetup.(name);
    nChannels = size(data,1);
    fprintf(fid,['        case ' num2str(Lengths(idxField)) ' %% ' name '\n']);
    fprintf(fid,['            nData = size(DataIn.' data{1,1} ',2);    \n']);
    fprintf(fid,['            DataOut = zeros(' num2str(nChannels) ',nData);\n']);
    for idxChannel = 1:nChannels;
        fprintf(fid,['            DataOut(' num2str(idxChannel) ',:) = DataIn.'...
            data{idxChannel,1} '; %% ' data{idxChannel,2} '\n']);
    end
end

fprintf(fid,'    otherwise \n');
fprintf(fid,'        error(''Invalid number of channels in input.'')\n');
fprintf(fid,'    end \n');


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Matrix -> Struct                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fprintf(fid,'    \n');
fprintf(fid,'else    \n');
fprintf(fid,'    %%Then converting from a matrix to a struct    \n');
fprintf(fid,'    switch size(DataIn,1)    \n');

for idxField=1:length(Fields)
    name = Fields{idxField};
    data = FileWritingSetup.(name);
    nChannels = size(data,1);
    fprintf(fid,['        case ' num2str(Lengths(idxField)) ' %% ' name '\n']);
    for idxChannel = 1:nChannels;
        fprintf(fid,['            DataOut.' data{idxChannel,1} ...
            ' = DataIn(' num2str(idxChannel) ',:); %% ' data{idxChannel,2} '\n']);
    end
end

fprintf(fid,'    otherwise \n');
fprintf(fid,'        error(''Invalid number of channels in input.'')\n');
fprintf(fid,'    end \n');

fprintf(fid,'    \n');
fprintf(fid,'end    \n');


fprintf(fid,'    \n');
fprintf(fid,'end    \n');
fclose(fid);
cd(CurrDir);

end
