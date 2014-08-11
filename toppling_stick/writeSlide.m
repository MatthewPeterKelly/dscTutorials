function writeSlide(Soln, Energy, name)

fid = fopen([ name '.m'],'w');
fprintf(fid,['function [dZ, C, E] = ' name '(~,Z,P)\n']);
fprintf(fid,'\n');
fprintf(fid,'%% DO NOT EDIT\n');
fprintf(fid,'%% This function was automatically generated\n');
fprintf(fid,'\n');
fprintf(fid,'u = P.u; %% Coefficient of friction\n');
fprintf(fid,'    dZ = zeros(size(Z));\n');
fprintf(fid,'    E = zeros(2,size(Z,2)); %% [potential; kinetic]\n');
fprintf(fid,'    C = zeros(2,size(Z,2)); %% [horizontal; vertical]\n');
fprintf(fid,'    \n');
fprintf(fid,'    m = P.m;\n');
fprintf(fid,'    g = P.g;\n');
fprintf(fid,'    L = P.L;\n');
fprintf(fid,'    I = P.I;\n');
fprintf(fid,'    \n');
fprintf(fid,'    th = Z(1,:);\n');
fprintf(fid,'    x = Z(2,:);\n');
fprintf(fid,'    dth = Z(3,:);\n');
fprintf(fid,'    dx = Z(4,:);\n');
fprintf(fid,'    \n');

fprintf(fid,'    dZ(1,:) = dth;\n');
fprintf(fid,'    dZ(2,:) = dx;\n');
fprintf(fid,'\n');
fprintf(fid,'if isinf(u)\n');
fprintf(fid,['    dZ(3,:) = ' vectorize(Soln.inf.ddth) ';\n']);
fprintf(fid,['    dZ(4,:) = ' vectorize(Soln.inf.ddx) ';\n']);
fprintf(fid,['    C(1,:) = ' vectorize(Soln.inf.H) ';\n']);
fprintf(fid,['    C(2,:) = ' vectorize(Soln.inf.V) ';\n\n']);

fprintf(fid,'elseif u==0\n');
fprintf(fid,['    dZ(3,:) = ' vectorize(Soln.zero.ddth) ';\n']);
fprintf(fid,['    dZ(4,:) = ' vectorize(Soln.zero.ddx) ';\n']);
fprintf(fid,['    C(1,:) = ' vectorize(Soln.zero.H) ';\n']);
fprintf(fid,['    C(2,:) = ' vectorize(Soln.zero.V) ';\n\n']);

fprintf(fid,'else\n');
fprintf(fid,'\n');

fprintf(fid,['    dZ(3,:) = ' vectorize(Soln.ddth) ';\n']);
fprintf(fid,['    dZ(4,:) = ' vectorize(Soln.ddx) ';\n']);
fprintf(fid,['    C(1,:) = ' vectorize(Soln.H) ';\n']);
fprintf(fid,['    C(2,:) = ' vectorize(Soln.V) ';\n\n']);

fprintf(fid,'\nend\n');
fprintf(fid,['    E(1,:) = ' vectorize(Energy.potential) ';\n']);
fprintf(fid,['    E(2,:) = ' vectorize(Energy.kinetic) ';\n']);

fprintf(fid,'end\n');
fclose(fid);

end