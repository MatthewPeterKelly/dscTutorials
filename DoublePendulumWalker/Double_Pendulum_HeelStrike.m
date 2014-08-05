function [Xp, Step_Vector, Slope] = Double_Pendulum_HeelStrike(Xm,J,P)          
                                                     
 % COMPUTER GENERATED FILE --  DO NOT EDIT!          
 %   See WriteHeelStrikeFile_Matlab.m for more details.     
                                                      
 %This file switches the feet for the double pendulum model of walking 
 %(point mass model only) and then does the impact map for the system). 
                                                      
th_m     = Xm(1,:);                                     
phi_m    = Xm(2,:);                                     
Dth_m    = Xm(3,:);                                     
Dphi_m   = Xm(4,:);                                     
                                                      
% Control inpulse on trailing foot                  
lJl    = J;   %Magnitude of impulse                 
                                                      
m1  = P.m1;                                          
m2  = P.m2;                                          
L  = P.L;                                          
                                                      
% Math happens here:                                    
gam = atan((sin(phi_m) + sin(th_m))./(cos(phi_m) + cos(th_m)));                                 
th_p = pi + 2.*gam - th_m;                              
phi_p = pi - 2.*gam + phi_m + 2.*th_m;                              
Dth_p = (lJl.*sin(phi_m - th_m) - Dth_m.*L.*m1.*cos(phi_m - th_m))./(L.*(m1 + m2 - m2.*cos(4.*gam - phi_m - 3.*th_m)^2));                              
Dphi_p = -(cos(4.*gam - phi_m - 3.*th_m).*(lJl.*sin(phi_m - th_m) - Dth_m.*L.*m1.*cos(phi_m - th_m)))./(L.*(m1 + m2 - m2.*cos(4.*gam - phi_m - 3.*th_m)^2));                              
                                                      
Step_X = L.*cos(phi_m) + L.*cos(th_m);                              
Step_Y = L.*sin(phi_m) + L.*sin(th_m);                              
                                                      
% Wrap the angles to make periodic constraints work  
AngleVec = WrapAngle([th_p; phi_p]);                   
                                                      
% Put things into vectors                           
Xp = [AngleVec; Dth_p; Dphi_p];                     
Step_Vector = [Step_X; Step_Y];                     
Slope = gam;                                        
                                                      
end                                                  
