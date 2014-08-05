function StructOutput = Make_Struct(varargin)

%This function is used to convert a list of matlab variables to elements of
%a struct.

N_Inputs = length(varargin);

for i=1:N_Inputs
    name = inputname(i);
    StructOutput.(name) = varargin{i};
end

end