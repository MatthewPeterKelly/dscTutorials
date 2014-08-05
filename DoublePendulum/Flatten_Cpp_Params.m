function Cpp_Params = Flatten_Cpp_Params(P_Cpp,Data)

%FUNCTION:
%   This function takes a state struct S, and collapses it to a column
%   vector s. It relies on information about the struct format that is
%   stored in P.StateStruct, which is created by the
%   Define_StateStruct_Data.m function.
%
%INPUTS:
%   P = the struct the is being used ito pass parameters
%   Data = contains information about P
%
%OUTPUTS:
%   param_vec = a column vector with all of the data in P
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Names = Data.Names;
Sizes = Data.Sizes;
Idx = Data.Idx;

Length = Idx(end,2);   %How many elements in the entire state
Cpp_Params = zeros(Length,1);   %Total size of the state struct
N = length(Names);   %total number of fields;
L = Sizes(:,1).*Sizes(:,2);   %Length of each field;

for i=1:N
   Cpp_Params(Idx(i,1):Idx(i,2)) = reshape(P_Cpp.(Names{i})',L(i),1);  %Transpose accounts for C++ array definition difference
end

end