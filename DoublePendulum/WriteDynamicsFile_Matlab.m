%WriteDynamicsFile

%
%This script is called by DeriveDynamicsDP.m
%

%FUNCTION: 
%   This function writes the dynamics file for double pendulum equation


%Make it so that vectorized calculations work
DDth_Soln = strrep(char(Soln.DDth),'*','.*');
DDphi_Soln = strrep(char(Soln.DDphi),'*','.*');
DDth_Soln = strrep(DDth_Soln,'^','.^');
DDphi_Soln = strrep(DDphi_Soln,'^','.^');
DDth_Soln = strrep(DDth_Soln,'/','./');
DDphi_Soln = strrep(DDphi_Soln,'/','./');

%Avoid evaluation of trig functions over and over again
DDth_Soln = strrep(DDth_Soln,'sin(th)','SinTh');
DDth_Soln = strrep(DDth_Soln,'cos(th)','CosTh');
DDth_Soln = strrep(DDth_Soln,'sin(phi)','SinPhi');
DDth_Soln = strrep(DDth_Soln,'cos(phi)','CosPhi');
DDth_Soln = strrep(DDth_Soln,'sin(phi - th)','SinPhiTh');
DDth_Soln = strrep(DDth_Soln,'cos(phi - th)','CosPhiTh');

DDphi_Soln = strrep(DDphi_Soln,'sin(th)','SinTh');
DDphi_Soln = strrep(DDphi_Soln,'cos(th)','CosTh');
DDphi_Soln = strrep(DDphi_Soln,'sin(phi)','SinPhi');
DDphi_Soln = strrep(DDphi_Soln,'cos(phi)','CosPhi');
DDphi_Soln = strrep(DDphi_Soln,'sin(phi - th)','SinPhiTh');
DDphi_Soln = strrep(DDphi_Soln,'cos(phi - th)','CosPhiTh');


%Open the file
fid = fopen('Double_Pendulum_Dynamics.m','w'                          );


%Write the file
fprintf(fid, 'function dX=Double_Pendulum_Dynamics(~,X,U,P)          \n');
fprintf(fid, '                                                     \n');
fprintf(fid, '                                                      \n');
fprintf(fid, ' %% COMPUTER GENERATED FILE --  DO NOT EDIT!          \n');
fprintf(fid, ' %%   See WriteDynamicsFile_Matlab.m for more details.     \n');
fprintf(fid, '                                                      \n');
fprintf(fid, 'th     = X(1,:);                                     \n');
fprintf(fid, 'phi    = X(2,:);                                     \n');
fprintf(fid, 'Dth    = X(3,:);                                     \n');
fprintf(fid, 'Dphi   = X(4,:);                                     \n');
fprintf(fid, '                                                     \n');
fprintf(fid, 'M1   = U(1,:);                                       \n');
fprintf(fid, 'M2   = U(2,:);                                       \n');
fprintf(fid, 'F1_x = U(3,:);                                       \n');
fprintf(fid, 'F1_y = U(4,:);                                       \n');
fprintf(fid, 'F2_x = U(5,:);                                       \n');
fprintf(fid, 'F2_y = U(6,:);                                       \n');
fprintf(fid, '                                                     \n');
fprintf(fid, 'g   = P.g;                                           \n');
fprintf(fid, 'L   = P.L;                                           \n');
fprintf(fid, 'm1  = P.m1;                                          \n');
fprintf(fid, 'm2  = P.m2;                                          \n');
fprintf(fid, '                                                     \n');
fprintf(fid, 'SinTh = sin(th);                                     \n');
fprintf(fid, 'CosTh = cos(th);                                     \n');
fprintf(fid, 'SinPhi = sin(phi);                                   \n');
fprintf(fid, 'CosPhi = cos(phi);                                   \n');
fprintf(fid, 'SinPhiTh = sin(phi-th);                              \n');
fprintf(fid, 'CosPhiTh = cos(phi-th);                              \n');
fprintf(fid, '                                                     \n');
fprintf(fid, ['DDth = ' DDth_Soln ';                              \n']);
fprintf(fid, ['DDphi = ' DDphi_Soln ';                            \n']);
fprintf(fid, '                                                     \n');
fprintf(fid, 'dX = [Dth; Dphi; DDth; DDphi];                      \n');
fprintf(fid, '                                                     \n');
fprintf(fid,'end                                                   \n');


%Close the file!
fclose(fid);
