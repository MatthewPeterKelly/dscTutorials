%WriteKineticsFile

%
%This script is called by DeriveDynamicsDP.m
%

%FUNCTION:
%   This file retuns all of the kinematics info for the system (pos,vel,acc)
%   It also computes the potential, kinetic and total mechanical energy of
%   the system


%% Convert everything to a character array
r_P1_O_CHAR_X = char(r_P1_O(1));
v_P1_O_CHAR_X = char(v_P1_O(1));
a_P1_O_CHAR_X = char(a_P1_O(1));

r_P2_O_CHAR_X = char(r_P2_O(1));
v_P2_O_CHAR_X = char(v_P2_O(1));
a_P2_O_CHAR_X = char(a_P2_O(1));

r_P1_O_CHAR_Y = char(r_P1_O(2));
v_P1_O_CHAR_Y = char(v_P1_O(2));
a_P1_O_CHAR_Y = char(a_P1_O(2));

r_P2_O_CHAR_Y = char(r_P2_O(2));
v_P2_O_CHAR_Y = char(v_P2_O(2));
a_P2_O_CHAR_Y = char(a_P2_O(2));



%% String Replacements
% Searches for IN and replaces it with OUT

%Vectorized calculations
IN = '*'; OUT = '.*'; Kinematics_String_Replace;   
IN = '^'; OUT = '.^'; Kinematics_String_Replace;
IN = '/'; OUT = './'; Kinematics_String_Replace;

%Don't do so many trig evals
IN = 'sin(th)'; OUT = 'SinTh'; Kinematics_String_Replace;
IN = 'cos(th)'; OUT = 'CosTh'; Kinematics_String_Replace;
IN = 'sin(phi)'; OUT = 'SinPhi'; Kinematics_String_Replace;
IN = 'cos(phi)'; OUT = 'CosPhi'; Kinematics_String_Replace;  

%Open file       
fid = fopen('Double_Pendulum_Kinematics.m','w'                        );


%Write file
fprintf(fid, 'function [K, E] = Double_Pendulum_Kinematics(X,P,U)         \n');
fprintf(fid, '                                                      \n');
fprintf(fid, ' %% COMPUTER GENERATED FILE --  DO NOT EDIT!          \n');
fprintf(fid, ' %%   See WriteKinematicsFile_Matlab.m for more details.     \n');
fprintf(fid, '                                                      \n');
fprintf(fid, ' %%READ ME :                                          \n');
fprintf(fid, ' %% Naming Conventions:                               \n');
fprintf(fid, ' %%     r = position                                  \n');
fprintf(fid, ' %%     v = velocity                                  \n');
fprintf(fid, ' %%     a = acceleration                              \n');
fprintf(fid, ' %%     O = Origin                                    \n');
fprintf(fid, ' %%      P1 = Joint between links 1 and 2              \n');
fprintf(fid, ' %%     P2 = Free end of link 2                       \n');
fprintf(fid, ' %%  Notes:                                            \n');
fprintf(fid, ' %%  - All fields are (2xN) matricies                  \n');
fprintf(fid, ' %%  - First row = horizontal comonent                 \n');
fprintf(fid, ' %%  - Second row = vertical comonent                  \n');
fprintf(fid, ' %%  - _A_B  reads of A with respect to B              \n');
fprintf(fid, ' %%  FIELDS:                                           \n');
fprintf(fid, ' %%    K.r_P1_O                                          \n');
fprintf(fid, ' %%    K.v_P1_O                                          \n');
fprintf(fid, ' %%    K.a_P1_O                                          \n');
fprintf(fid, ' %%                                                         \n');
fprintf(fid, ' %%    K.r_P2_O                                          \n');
fprintf(fid, ' %%    K.v_P2_O                                          \n');
fprintf(fid, ' %%    K.a_P2_O                                          \n');
fprintf(fid, '                                                      \n');
fprintf(fid, ' %%    E.Potential                                     \n');
fprintf(fid, ' %%    E.Kinetic                                       \n');
fprintf(fid, ' %%    E.Total                                          \n');
fprintf(fid, '                                                     \n');
fprintf(fid, '                                                     \n');
fprintf(fid, 'dX =  Double_Pendulum_Dynamics(0,X,U,P);              \n');
fprintf(fid, '                                                     \n');
fprintf(fid, 'th     =  X(1,:);                                    \n');
fprintf(fid, 'Dth    =  X(3,:);                                    \n');
fprintf(fid, 'DDth   = dX(3,:);                                    \n');
fprintf(fid, 'phi    =  X(2,:);                                    \n');
fprintf(fid, 'Dphi   =  X(4,:);                                    \n');
fprintf(fid, 'DDphi  = dX(4,:);                                    \n');
fprintf(fid, '                                                     \n');
fprintf(fid, 'g   = P.g;                                           \n');
fprintf(fid, 'L   = P.L;                                           \n');
fprintf(fid, 'm1  = P.m1;                                          \n');
fprintf(fid, 'm2  = P.m2;                                          \n');
fprintf(fid, '                                                     \n');
fprintf(fid, '%% Reduce the number of trig evaluations             \n');
fprintf(fid, 'SinTh = sin(th);                                     \n');
fprintf(fid, 'CosTh = cos(th);                                     \n');
fprintf(fid, 'SinPhi = sin(phi);                                   \n');
fprintf(fid, 'CosPhi = cos(phi);                                   \n');
fprintf(fid, '                                                     \n');
fprintf(fid, '                                                     \n');
fprintf(fid, '%% Initialize the output:                            \n');
fprintf(fid, 'N = length(th);                                      \n');
fprintf(fid, 'K.r_P1_O = zeros(2,N);                              \n');
fprintf(fid, 'K.v_P1_O = zeros(2,N);                              \n');
fprintf(fid, 'K.a_P1_O = zeros(2,N);                              \n');
fprintf(fid, '                                                     \n');
fprintf(fid, 'K.r_P2_O = zeros(2,N);                              \n');
fprintf(fid, 'K.v_P2_O = zeros(2,N);                              \n');
fprintf(fid, 'K.a_P2_O = zeros(2,N);                              \n');
fprintf(fid, '                                                     \n');
fprintf(fid, '                                                     \n');
fprintf(fid, ['K.r_P1_O(1,:) = ' r_P1_O_CHAR_X ';                  \n']);
fprintf(fid, ['K.v_P1_O(1,:) = ' v_P1_O_CHAR_X ';                  \n']);
fprintf(fid, ['K.a_P1_O(1,:) = ' a_P1_O_CHAR_X ';                  \n']);
fprintf(fid, '                                                     \n');
fprintf(fid, ['K.r_P2_O(1,:) = ' r_P2_O_CHAR_X ';                  \n']);
fprintf(fid, ['K.v_P2_O(1,:) = ' v_P2_O_CHAR_X ';                  \n']);
fprintf(fid, ['K.a_P2_O(1,:) = ' a_P2_O_CHAR_X ';                  \n']);
fprintf(fid, '                                                     \n');
fprintf(fid, '                                                     \n');
fprintf(fid, ['K.r_P1_O(2,:) = ' r_P1_O_CHAR_Y ';                  \n']);
fprintf(fid, ['K.v_P1_O(2,:) = ' v_P1_O_CHAR_Y ';                  \n']);
fprintf(fid, ['K.a_P1_O(2,:) = ' a_P1_O_CHAR_Y ';                  \n']);
fprintf(fid, '                                                     \n');
fprintf(fid, ['K.r_P2_O(2,:) = ' r_P2_O_CHAR_Y ';                  \n']);
fprintf(fid, ['K.v_P2_O(2,:) = ' v_P2_O_CHAR_Y ';                  \n']);
fprintf(fid, ['K.a_P2_O(2,:) = ' a_P2_O_CHAR_Y ';                  \n']);
fprintf(fid, '                                                     \n');
fprintf(fid, '                                                     \n');
fprintf(fid,'%% Energy Calculations:                                       \n');
fprintf(fid, 'E.Potential = m1*g*K.r_P1_O(2,:) + m2*g*K.r_P2_O(2,:);       \n');
fprintf(fid, '                                                             \n');
fprintf(fid, 'Speed_Squared_1 = K.v_P1_O(1,:).^2 + K.v_P1_O(2,:).^2;       \n');
fprintf(fid, 'Speed_Squared_2 = K.v_P2_O(1,:).^2 + K.v_P2_O(2,:).^2;       \n');
fprintf(fid, 'E.Kinetic = 0.5*m1*Speed_Squared_1 + 0.5*m2*Speed_Squared_2; \n');
fprintf(fid, '                                                             \n');
fprintf(fid, 'E.Total = E.Potential + E.Kinetic;                           \n');
fprintf(fid, '                                                     \n');
fprintf(fid, '                                                     \n');
fprintf(fid,'end                                                   \n');

%Close the file!
fclose(fid);

