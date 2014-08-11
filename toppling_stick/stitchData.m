function [D, Jumps, JumpIdx] = stitchData(Data)

%This function goes through and stitches all of the data together into a
%single time series for plotting.

%Get the size of things
N = length(Data);  %Number of phases
Jumps = zeros(N+1,1);
JumpIdx = zeros(N+1,1);
Jumps(1) = Data(1).time(1);
JumpIdx(1) = 1;
D.phase = cell(N,1);
IdxBnd = zeros(N,2);
n = 0;   %Number of data points
for i=1:N
    n = n+length(Data(i).time);
    Jumps(i+1) = Data(i).time(end);
    JumpIdx(i+1) = n;
    D.phase{i} = Data(i).phase;
    IdxBnd(i,1) = 1;
    IdxBnd(i,2) = length(Data(i).time);
end
for i=2:N
    IdxBnd(i,:) = IdxBnd(i,:) + IdxBnd(i-1,2);
end

%Initialize memory
D.time = zeros(n,1);
Names = {'state','contact','energy'};
for i=1:length(Names)
    D.(Names{i}) = structInit(Data(1).(Names{i}),n);
end

%Populate the data structure
for i=1:N
    IdxList = IdxBnd(i,1):IdxBnd(i,2);
    D.time(IdxList) = Data(i).time;
    for j=1:length(Names)
        subNames = fieldnames(D.(Names{j}));
        for k=1:length(subNames)
            D.(Names{j}).(subNames{k})(IdxList) = ...
                Data(i).(Names{j}).(subNames{k});
        end
    end
end

end

function S = structInit(Struct,n)

names = fieldnames(Struct);
for i=1:length(names)
    S.(names{i}) = zeros(n,1);
end

end