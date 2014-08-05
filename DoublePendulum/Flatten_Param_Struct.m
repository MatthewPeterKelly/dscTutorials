function param_vec = Flatten_Param_Struct(P_Input)
%
% param_vec = Flatten_Param_Struct(P_Input)
%
%FUNCTION:  
%   
%   This function takes a parameter struct P_Input and collapses it to a
%   vector of parameters param_vec in the exact same way as in the function
%   Write_Parameter_File.m. This function is largely copy-pasted from that
%   function. Please reference that for more detailed usage files.
%
%   This function does not do error checking on the struct (to increase
%   speed) so make sure to run it through WRITE_PARAMETER_FILE first.
%   
%   See also  WRITE_PARAMETER_FILE  PARAMSTRUCTDATA  FLATTEN_CPP_PARAMS



%Check the data type for each field
Names = sort(fieldnames(P_Input));   %Get the names of each field, and put them in order for repeatability

%Loop through each field
for i=1:length(Names)   
    
   if isnumeric(P_Input.(Names{i}))   %Then the field is purely numeric. Woo!
       P.(Names{i}) = P_Input.(Names{i});   %This is a numeric valued field - OK to pass to C++
   
   elseif isstruct(P_Input.(Names{i}))   %Now things get complicated
        
        %Then it is a struct - recursively call this function
        %Then write a new field that represents that flattened struct --
        Flattened_Struct = Flatten_Param_Struct(P_Input.(Names{i}));   %RECURSION
          
        %Now store the flattened struct
        P.(Names{i}) = Flattened_Struct;   %Now it is stored as a matrix
     
   else
       %Not a numeric value or a struct - not supported - Display a warning
       disp(' ')
       disp('~~~~ WARNING -- Write_Parameter_File.m')
       disp(['~~~~    P.' Names{i} ' is not supported. This may cause an error.'])
          
   end
end   
    
%% Take the formated struct and collapse it to a vector
%IMPORTANT TWO LINES!!!

%This stores the information about each field so that it can be used later
%to assign names to each element of the parameter struct
Data = ParamStructData(P);  %used in later sections as well

%This function takes the information in Data and uses it to collapse the
%struct into a single vector
param_vec = Flatten_Cpp_Params(P,Data);

end