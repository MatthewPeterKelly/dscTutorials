function [X_Out, StructDat] = convertVecStruct(X_In,StructDat)
%
%[X_Out, StructDat] = convertVecStruct(X_In,StructDat)
%
%This function is used to flatten a state struct to a vector or vice versa.
%Each field of the struct must contain a matrix (or scalar) of doubles.
%
%INPUTS:
% X_In = either a vector or struct of data
% StructDat = a struct with information about the fields in X_In. On the
%     first call with X_In = Struct, this field should be ommitted or [].
%     After the first call, the Struct -> vector conversion is faster if
%     this argument is included.
%
%OUTPUTS:
% X_Out = either a vector or struct - opposite of the input
% StructDat = a struct with information about the fields in X
%
%
%METHOD:
% If the input is a struct, then this algorithm takes each field and uses
% the matlab reshape command to make the field data into a column vector.
% The algorithm then takes all of the column vectors and combines them into
% one bit long column vector that is returned as x
%
% If the input is a vector, then the algorithm breaks the vector into a
% column vector corresponding to each field, and then uses reshape to get
% each of these column vectors into the correct shape, which is then
% assigned to a field in the return struct with the correct name.
%

if isstruct(X_In)
  %Then the input is a struct - goal is to produce a vector
  if nargin==1 || isempty(StructDat)
      %Then we don't know the struct data yet - compute it
      StructDat.FieldNames = fieldnames(X_In);
      N_Fields = length(StructDat.FieldNames);
      StructDat.FieldSize = zeros(N_Fields,2);
      for i=1:N_Fields
        StructDat.FieldSize(i,:) = size(X_In.(StructDat.FieldNames{i}));
      end  
  end
  
  %Get the number of elements in the data structure
  N_elements = StructDat.FieldSize(:,1).*StructDat.FieldSize(:,2);
  N_Fields= length(N_elements);
  
  X_Out = zeros(sum(N_elements),1);  %Initialize the vector
  
  %Store each field of the struct into the vector
  IdxLow = 1;
  for i=1:N_Fields
    IdxUpp = IdxLow-1+N_elements(i);
    fieldName = StructDat.FieldNames{i};
    X_Out(IdxLow:IdxUpp) = reshape(X_In.(fieldName),N_elements(i),1);
    IdxLow = IdxUpp+1;
  end  
  
else
  %Then the input is a vector - goal is to produce a struct
  
  %Get the number of elements in the data structure
  N_elements = StructDat.FieldSize(:,1).*StructDat.FieldSize(:,2);
  N_Fields = length(N_elements);
  
  %Store each field of the struct into the vector
  IdxLow = 1;
  for i=1:N_Fields
    IdxUpp = IdxLow-1+N_elements(i);
    fieldName = StructDat.FieldNames{i};
    data = X_In(IdxLow:IdxUpp);
    N = StructDat.FieldSize(i,1);
    M = StructDat.FieldSize(i,2);
    X_Out.(fieldName) = reshape(data,N,M);
    IdxLow = IdxUpp+1;
  end  
  
end


end




% An alternate forumlation that uses fewer lines of code:

% % % % function [X_Out, StructDat] = Convert_State(X_In,StructDat)
% % % % %
% % % % %[X_Out, StructDat] = Convert_State(X_In,StructDat)
% % % % %
% % % % %This function is used to flatten a state struct to a vector or vice versa.
% % % % %Each field of the struct must contain a matrix (or scalar) of doubles.
% % % % %
% % % % %INPUTS:
% % % % % X_In = either a vector or struct of data
% % % % % StructDat = a struct with information about the fields in X_In. On the
% % % % %     first call with X_In = Struct, this field should be ommitted or [].
% % % % %     After the first call, the Struct -> vector conversion is faster if
% % % % %     this argument is included.
% % % % %
% % % % %OUTPUTS:
% % % % % X_Out = either a vector or struct - opposite of the input
% % % % % StructDat = a struct with information about the fields in X
% % % % %
% % % % 
% % % % if isstruct(X_In)
% % % %   %Then the input is a struct - goal is to produce a vector
% % % %   if nargin==1 || isempty(StructDat)
% % % %       %Then we don't know the struct data yet - compute it
% % % %       StructDat.FieldNames = fieldnames(X_In);
% % % %       N_Fields = length(StructDat.FieldNames);
% % % %       StructDat.FieldSize = zeros(N_Fields,2);
% % % %       StructDat.FieldIdx = zeros(N_Fields,2);
% % % %       IdxLow = 1;
% % % %       for i=1:N_Fields
% % % %         [N,M] = size(X_In.(StructDat.FieldNames{i}));
% % % %         StructDat.FieldSize(i,:) = [N,M];
% % % %         IdxUpp = IdxLow-1+N*M;
% % % %         StructDat.FieldIdx(i,:) = [IdxLow, IdxUpp];
% % % %         IdxLow = IdxUpp+1;
% % % %       end  
% % % %   end
% % % %   
% % % %   %Get the number of elements in the data structure
% % % %   N_elements = StructDat.FieldSize(:,1).*StructDat.FieldSize(:,2);
% % % %   N_Fields = length(N_elements);
% % % %   
% % % %   X_Out = zeros(sum(N_elements),1);  %Initialize the vector
% % % %   
% % % %   %Store each field of the struct into the vector
% % % %   for i=1:N_Fields
% % % %     Idx = StructDat.FieldIdx(i,1):StructDat.FieldIdx(i,2);
% % % %     X_Out(Idx) = reshape(X_In.(StructDat.FieldNames{i}),N_elements(i),1);
% % % %   end  
% % % %   
% % % % else
% % % %   %Then the input is a vector - goal is to produce a struct
% % % %   
% % % %   %Get the number of elements in the data structure
% % % %   N_elements = StructDat.FieldSize(:,1).*StructDat.FieldSize(:,2);
% % % %   N_Fields = length(N_elements);
% % % %   
% % % %   %Store each field of the struct into the vector
% % % %   for i=1:N_Fields
% % % %     Idx = StructDat.FieldIdx(i,1):StructDat.FieldIdx(i,2);
% % % %     N = StructDat.FieldSize(i,1);
% % % %     M = StructDat.FieldSize(i,2);
% % % %     X_Out.(StructDat.FieldNames{i}) = reshape(X_In(Idx),N,M);
% % % %   end  
% % % %   
% % % % end
% % % % 
% % % % 
% % % % end