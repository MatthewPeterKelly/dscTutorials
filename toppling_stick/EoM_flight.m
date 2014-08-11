function EoM_flight()
%
%Derives the equations of motion for a rigid body moving in 2D space
%
%

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                             Parameters                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

syms m I L g 'real'
% m = mass of the rigid object
% I = moment of inertia about the center of mass
% L = distance between the tip of the object (O) and center of mass (G)
% g = acceleration due to gravity

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        States and Unknowns                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

syms th x y dth dx dy 'real'
% th = angle between [position of G wrt O] and positive vertical axis
% x = horizontal position of point O
% y = vertical position of point O
% dth = time derivative of th
% dx = time derivative of x
% dy = time derivative of y

syms ddth ddx ddy 'real'
% ddth = second time derivative of th
% ddx = second time derivative of x
% ddy = second time derivative of y

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Coordinate System                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Inertial reference frame
i = sym([1;0;0]);
j = sym([0;1;0]);
k = sym([0;0;1]);

%Rotating reference frame
er = cos(th)*j + sin(th)*(-i);   %Unit vector along the line G wrt O
et = -sin(th)*j + cos(th)*(-i);  %Unit vector normal to er in the plane

%Frame derivatives
der = dth*et;
det = -dth*er;
dder = ddth*et + dth*det;

%Position vectors;
rg = x*i + y*j + L*er;
drg = dx*i + dy*j + L*der;
ddrg = ddx*i + ddy*j + L*dder;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%           Conservation of angular momentum rate about G                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% sum of moments = rate of change in angular momentum
Mo = sym([0;0;0]);   %Sum of moments about G
dHo = I*ddth;   %Rate of change of angular momentum
Eqn1 = simplify(dot(Mo-dHo,k));    %Extract k component and simplify

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Conservation of linear momentum rate                    %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% sum of forces = rate of change in momentum
sum_forces = -m*g*j;
momentum_rate = m*ddrg;
eqn = simplify(sum_forces-momentum_rate);
Eqn2 = dot(eqn,i);
Eqn3 = dot(eqn,j);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Mechanical Energy of the system                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

Energy.potential = -dot(rg,-m*g*j);
Energy.kinetic = (1/2)*m*dot(drg,drg) + (1/2)*I*dth^2;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Solve equations of motion                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

Soln = solve(Eqn1,Eqn2,Eqn3,ddth,ddx,ddy);
Soln.ddth = simplify(Soln.ddth);
Soln.ddx = simplify(Soln.ddx);
Soln.ddy = simplify(Soln.ddy);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Write Function File                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fid = fopen('dynamics_flight.m','w');
fprintf(fid,'function [dZ, E] = dynamics_flight(~,Z,P)\n');
fprintf(fid,'\n');
fprintf(fid,'%% DO NOT EDIT\n');
fprintf(fid,'%% This function was automatically generated\n');
fprintf(fid,'\n');
fprintf(fid,'th = Z(1,:);\n');
fprintf(fid,'x = Z(2,:);\n');
fprintf(fid,'y = Z(3,:);\n');
fprintf(fid,'dth = Z(4,:);\n');
fprintf(fid,'dx = Z(5,:);\n');
fprintf(fid,'dy = Z(6,:);\n');
fprintf(fid,'\n');
fprintf(fid,'m = P.m;\n');
fprintf(fid,'g = P.g;\n');
fprintf(fid,'L = P.L;\n');
fprintf(fid,'I = P.I;\n');
fprintf(fid,'\n');
fprintf(fid,'dZ = zeros(size(Z));\n');
fprintf(fid,'dZ(1,:) = dth;\n');
fprintf(fid,'dZ(2,:) = dx;\n');
fprintf(fid,'dZ(3,:) = dy;\n');
fprintf(fid,['dZ(4,:) = ' vectorize(Soln.ddth) ';\n']);
fprintf(fid,['dZ(5,:) = ' vectorize(Soln.ddx) ';\n']);
fprintf(fid,['dZ(6,:) = ' vectorize(Soln.ddy) ';\n']);
fprintf(fid,'\n');
fprintf(fid,'%% Energy: \n');
fprintf(fid,'E = zeros(2,length(th)); %% [potential; kinetic]\n');  
fprintf(fid,['E(1,:) = ' vectorize(Energy.potential) ';\n']);
fprintf(fid,['E(2,:) = ' vectorize(Energy.kinetic) ';\n']);
fprintf(fid,'\nend\n');
fclose(fid);

end