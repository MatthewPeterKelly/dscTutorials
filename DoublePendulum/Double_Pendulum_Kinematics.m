function [K, E] = Double_Pendulum_Kinematics(X,P,U)         
                                                      
 % COMPUTER GENERATED FILE --  DO NOT EDIT!          
 %   See WriteKinematicsFile_Matlab.m for more details.     
                                                      
 %READ ME :                                          
 % Naming Conventions:                               
 %     r = position                                  
 %     v = velocity                                  
 %     a = acceleration                              
 %     O = Origin                                    
 %      P1 = Joint between links 1 and 2              
 %     P2 = Free end of link 2                       
 %  Notes:                                            
 %  - All fields are (2xN) matricies                  
 %  - First row = horizontal comonent                 
 %  - Second row = vertical comonent                  
 %  - _A_B  reads of A with respect to B              
 %  FIELDS:                                           
 %    K.r_P1_O                                          
 %    K.v_P1_O                                          
 %    K.a_P1_O                                          
 %                                                         
 %    K.r_P2_O                                          
 %    K.v_P2_O                                          
 %    K.a_P2_O                                          
                                                      
 %    E.Potential                                     
 %    E.Kinetic                                       
 %    E.Total                                          
                                                     
                                                     
dX =  Double_Pendulum_Dynamics(0,X,U,P);              
                                                     
th     =  X(1,:);                                    
Dth    =  X(3,:);                                    
DDth   = dX(3,:);                                    
phi    =  X(2,:);                                    
Dphi   =  X(4,:);                                    
DDphi  = dX(4,:);                                    
                                                     
g   = P.g;                                           
L   = P.L;                                           
m1  = P.m1;                                          
m2  = P.m2;                                          
                                                     
% Reduce the number of trig evaluations             
SinTh = sin(th);                                     
CosTh = cos(th);                                     
SinPhi = sin(phi);                                   
CosPhi = cos(phi);                                   
                                                     
                                                     
% Initialize the output:                            
N = length(th);                                      
K.r_P1_O = zeros(2,N);                              
K.v_P1_O = zeros(2,N);                              
K.a_P1_O = zeros(2,N);                              
                                                     
K.r_P2_O = zeros(2,N);                              
K.v_P2_O = zeros(2,N);                              
K.a_P2_O = zeros(2,N);                              
                                                     
                                                     
K.r_P1_O(1,:) = L.*CosTh;                  
K.v_P1_O(1,:) = -Dth.*L.*SinTh;                  
K.a_P1_O(1,:) = -L.*(DDth.*SinTh + Dth.^2.*CosTh);                  
                                                     
K.r_P2_O(1,:) = L.*CosPhi + L.*CosTh;                  
K.v_P2_O(1,:) = - Dphi.*L.*SinPhi - Dth.*L.*SinTh;                  
K.a_P2_O(1,:) = - L.*(DDphi.*SinPhi + Dphi.^2.*CosPhi) - L.*(DDth.*SinTh + Dth.^2.*CosTh);                  
                                                     
                                                     
K.r_P1_O(2,:) = L.*SinTh;                  
K.v_P1_O(2,:) = Dth.*L.*CosTh;                  
K.a_P1_O(2,:) = L.*(DDth.*CosTh - Dth.^2.*SinTh);                  
                                                     
K.r_P2_O(2,:) = L.*SinPhi + L.*SinTh;                  
K.v_P2_O(2,:) = Dphi.*L.*CosPhi + Dth.*L.*CosTh;                  
K.a_P2_O(2,:) = L.*(DDphi.*CosPhi - Dphi.^2.*SinPhi) + L.*(DDth.*CosTh - Dth.^2.*SinTh);                  
                                                     
                                                     
% Energy Calculations:                                       
E.Potential = m1*g*K.r_P1_O(2,:) + m2*g*K.r_P2_O(2,:);       
                                                             
Speed_Squared_1 = K.v_P1_O(1,:).^2 + K.v_P1_O(2,:).^2;       
Speed_Squared_2 = K.v_P2_O(1,:).^2 + K.v_P2_O(2,:).^2;       
E.Kinetic = 0.5*m1*Speed_Squared_1 + 0.5*m2*Speed_Squared_2; 
                                                             
E.Total = E.Potential + E.Kinetic;                           
                                                     
                                                     
end                                                   
