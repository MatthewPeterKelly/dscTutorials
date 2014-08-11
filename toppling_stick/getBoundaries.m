function Bnd = getBoundaries(xx,yy,zz)

%This function returns a collection of lines for plotting the boundaries
%between the sections in DD (a matrix of integers).

[N,M] = size(zz);
Bnd = [];
for i=1:N
   x = xx(i,:);
   y = yy(i,:);
   z = zz(i,:);
   J = getJumps(x,y,z);
   Bnd = [Bnd;J];
end

for j=1:M
   x = xx(:,j);
   y = yy(:,j);
   z = zz(:,j);
   J = getJumps(x,y,z);
   Bnd = [Bnd;J];
end

end

function J = getJumps(x,y,z)

% in = coordinate
% out = class

idxJ = diff(z)~=0;

low = x(1:(end-1)); low = low(idxJ);
upp = x(2:end); upp = upp(idxJ);
Jx = 0.5*(low+upp);

low = y(1:(end-1)); low = low(idxJ);
upp = y(2:end); upp = upp(idxJ);
Jy = 0.5*(low+upp);

J = [Jx(:), Jy(:)];

end

