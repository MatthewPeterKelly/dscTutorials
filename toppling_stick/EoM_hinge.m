function EoM_hinge()
%
%Derives the equations of motion for a rigid body rotating in 2D about a
%single fixed point (hinge).
%
%

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                             Parameters                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

syms m I L g 'real'
% m = mass of the rigid object
% I = moment of inertia about the center of mass
% L = distance between the hinge (O) and center of mass (G)
% g = acceleration due to gravity

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        States and Unknowns                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

syms th dth 'real'
% th = angle between [position of G wrt O] and positive vertical axis
% dth = time derivative of th
syms ddth H V 'real'
% ddth = second time derivative of th
% H = horizontal component of the contact force
% V = vertical component of the contact force

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

%Position 
rg = L*er;
drg = L*der;
ddrg = L*dder;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%           Conservation of angular momentum rate about hinge             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% sum of moments = rate of change in angular momentum
Mo = cross(rg,-m*g*j);   %Sum of moments about hinge
dHo = I*ddth + cross(rg,m*ddrg);   %Rate of change of angular momentum
Eqn1 = simplify(dot(Mo-dHo,k));    %Extract k component and simplify

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Conservation of linear momentum rate                    %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% sum of forces = rate of change in momentum
sum_forces = H*i + V*j - m*g*j;
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

Soln = solve(Eqn1,Eqn2,Eqn3,ddth,V,H);
Soln.ddth = simplify(Soln.ddth);
Soln.H = simplify(Soln.H);
Soln.V = simplify(Soln.V);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Write Function File                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fid = fopen('dynamics_hinge.m','w');
fprintf(fid,'function [dZ, C, E] = dynamics_hinge(~,Z,P)\n');
fprintf(fid,'\n');
fprintf(fid,'%% DO NOT EDIT\n');
fprintf(fid,'%% This function was automatically generated\n');
fprintf(fid,'\n');
fprintf(fid,'th = Z(1,:);\n');
fprintf(fid,'dth = Z(2,:);\n');
fprintf(fid,'\n');
fprintf(fid,'m = P.m;\n');
fprintf(fid,'g = P.g;\n');
fprintf(fid,'L = P.L;\n');
fprintf(fid,'I = P.I;\n');
fprintf(fid,'\n');
fprintf(fid,'dZ = zeros(size(Z));\n');
fprintf(fid,'dZ(1,:) = dth;\n');
fprintf(fid,['dZ(2,:) = ' vectorize(Soln.ddth) ';\n']);
fprintf(fid,'\n');
fprintf(fid,'%% Contact Forces: \n');
fprintf(fid,'C = zeros(2,length(th)); %% [horizontal; vertical]\n');  
fprintf(fid,['C(1,:) = ' vectorize(Soln.H) ';\n']);
fprintf(fid,['C(2,:) = ' vectorize(Soln.V) ';\n\n']);
fprintf(fid,'%% Energy: \n');
fprintf(fid,'E = zeros(2,length(th)); %% [potential; kinetic]\n');  
fprintf(fid,['E(1,:) = ' vectorize(Energy.potential) ';\n']);
fprintf(fid,['E(2,:) = ' vectorize(Energy.kinetic) ';\n']);
fprintf(fid,'\nend\n');
fclose(fid);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Analytic Analysis                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

Energy.total = Energy.potential + Energy.kinetic;

%%%% Get an equation that gives angular rate as a function of angle, given
%%%% that the system starts inverted at rest.
IC.zero = simplify(limit(Energy.total,th,0));
IC.zero = simplify(limit(IC.zero,dth,0));
IC.expr = Energy.total - IC.zero;
IC.dth = simplify(solve(IC.expr,dth));

%%%% Now get an equation that gives the horizontal and vertical contact
%%%% forces for those initial conditions
IC.V = simplify(subs(Soln.V,dth,IC.dth));
IC.H = simplify(subs(Soln.H,dth,IC.dth));

%%%% Write a function to evaluate these functions
fid = fopen('topple_contactVertical.m','w');
fprintf(fid,'function V = topple_contactVertical(th,P)\n');
fprintf(fid,'\n');
fprintf(fid,'%% AUTOMATICALLY GENERATED  --  DO NOT EDIT\n');
fprintf(fid,'%% Computes vertical component of contact force for a stick toppling from rest.\n');
fprintf(fid,'\n');
fprintf(fid,'m = P.m;\n');
fprintf(fid,'g = P.g;\n');
fprintf(fid,'L = P.L;\n');
fprintf(fid,'I = P.I;\n');
fprintf(fid,'\n');
fprintf(fid,['V = ' vectorize(IC.V(1)) ';\n']);
fprintf(fid,'\n');
fprintf(fid,'end\n');
fclose(fid);

fid = fopen('topple_contactHorizontal.m','w');
fprintf(fid,'function H = topple_contactHorizontal(th,P)\n');
fprintf(fid,'\n');
fprintf(fid,'%% AUTOMATICALLY GENERATED  --  DO NOT EDIT\n');
fprintf(fid,'%% Computes horizontal component of contact force for a stick toppling from rest.\n');
fprintf(fid,'\n');
fprintf(fid,'m = P.m;\n');
fprintf(fid,'g = P.g;\n');
fprintf(fid,'L = P.L;\n');
fprintf(fid,'I = P.I;\n');
fprintf(fid,'\n');
fprintf(fid,['H = ' vectorize(IC.H(1)) ';\n']);
fprintf(fid,'\n');
fprintf(fid,'end\n');
fclose(fid);

fid = fopen('topple_angularRate.m','w');
fprintf(fid,'function dth = topple_angularRate(th,P)\n');
fprintf(fid,'\n');
fprintf(fid,'%% AUTOMATICALLY GENERATED  --  DO NOT EDIT\n');
fprintf(fid,'%% Computes angular rate for a stick toppling from rest.\n');
fprintf(fid,'\n');
fprintf(fid,'m = P.m;\n');
fprintf(fid,'g = P.g;\n');
fprintf(fid,'L = P.L;\n');
fprintf(fid,'I = P.I;\n');
fprintf(fid,'\n');
fprintf(fid,['dth = sign(th).*' vectorize(IC.dth(1)) ';\n']);
fprintf(fid,'\n');
fprintf(fid,'end\n');
fclose(fid);

%%%% At what value of u does a point mass switch between falling forwards
%%%% and backwards?
MU.nom = simplify(-Soln.H/Soln.V);
MU.subs = simplify(subs(MU.nom,dth, IC.dth(1)));

fid = fopen('topple_criticalMu.m','w');
fprintf(fid,'function mu = topple_criticalMu(th,P)\n');
fprintf(fid,'\n');
fprintf(fid,'%% AUTOMATICALLY GENERATED  --  DO NOT EDIT\n');
fprintf(fid,'%% Computes angular rate for a stick toppling from rest.\n');
fprintf(fid,'\n');
fprintf(fid,'m = P.m;\n');
fprintf(fid,'g = P.g;\n');
fprintf(fid,'L = P.L;\n');
fprintf(fid,'I = P.I;\n');
fprintf(fid,'\n');
fprintf(fid,['mu = ' vectorize(MU.subs) ';\n']);
fprintf(fid,'\n');
fprintf(fid,'end\n');
fclose(fid);

end





