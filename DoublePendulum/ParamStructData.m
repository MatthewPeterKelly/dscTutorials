function Data = ParamStructData(S)

%FUNCTION:
%   This function takes a data struct, where every field contains a (N x M)
%   matrix of numbers, and store the names of each field, the size of each
%   field, and the total number of elements. This is then later used to
%   pack everything into a struct.
%
%INPUTS:
%   S = the data struct of interest. Each field must contain a matrix of
%       numbers (no strings or other weird data types)
%
%OUTUTS:
%   Data = struct with the added fields 
%       Data.Names = a (L x 1) cell array of field names
%       Data.Size = a (L x 2) matrix of the size of these fields
%       Data.Length = a scalar number of elements in S

    Names = fieldnames(S);
    Sizes = zeros(length(Names),2);
    Idx = zeros(length(Names),2);
    
    for i=1:length(Names)
       Sizes(i,:) = size(S.(Names{i})); 
    end
    
    %Figure out the start and end indicies of each field.
    L = Sizes(:,1).*Sizes(:,2);
    Idx(1,1) = 1;
    for i=2:length(L)
       Idx(i-1,2) = Idx(i-1,1)-1+L(i-1);
       Idx(i,1) = Idx(i-1,2) + 1;
    end
    Idx(end,2) = sum(L);
    
    %Store the results:
    Data.Names = Names;
    Data.Sizes = Sizes;
    Data.Idx = Idx;
    
end