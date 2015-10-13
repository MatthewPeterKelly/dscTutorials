function F = structBlkDiag(S,field)
% F = structBlkDiag(S,field)
%
% This function creates a block diagional matrix from a field of a struct
% array
%

n = length(S);
cmd = ['F' ' = blkdiag(']; 
for i=1:n
   cmd = [cmd 'S(' num2str(i) ').' field]; %#ok<AGROW>
   if i < n
      cmd = [cmd ', ']; %#ok<AGROW>
   else
       cmd = [cmd ');'];%#ok<AGROW>
   end
end

eval(cmd);

end