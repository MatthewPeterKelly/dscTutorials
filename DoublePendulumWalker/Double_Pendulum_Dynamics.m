function dX=Double_Pendulum_Dynamics(~,X,U,P)          
                                                     
                                                      
 % COMPUTER GENERATED FILE --  DO NOT EDIT!          
 %   See WriteDynamicsFile_Matlab.m for more details.     
                                                      
th     = X(1,:);                                     
phi    = X(2,:);                                     
Dth    = X(3,:);                                     
Dphi   = X(4,:);                                     
                                                     
M1   = U(1,:);                                       
M2   = U(2,:);                                       
F1_x = U(3,:);                                       
F1_y = U(4,:);                                       
F2_x = U(5,:);                                       
F2_y = U(6,:);                                       
                                                     
g   = P.g;                                           
L   = P.L;                                           
m1  = P.m1;                                          
m2  = P.m2;                                          
                                                     
SinTh = sin(th);                                     
CosTh = cos(th);                                     
SinPhi = sin(phi);                                   
CosPhi = cos(phi);                                   
SinPhiTh = sin(phi-th);                              
CosPhiTh = cos(phi-th);                              
                                                     
DDth = (2.*M1 - 2.*M2 - 2.*M2.*CosPhiTh - F2_y.*L.*cos(2.*phi - th) + F2_x.*L.*sin(2.*phi - th) + 2.*F1_y.*L.*CosTh + F2_y.*L.*CosTh - 2.*F1_x.*L.*SinTh - F2_x.*L.*SinTh + 2.*Dphi.^2.*L.^2.*m2.*SinPhiTh - 2.*L.*g.*m1.*CosTh - L.*g.*m2.*CosTh + Dth.^2.*L.^2.*m2.*sin(2.*phi - 2.*th) + L.*g.*m2.*cos(2.*phi - th))./(L.^2.*(2.*m1 + m2 - m2.*cos(2.*phi - 2.*th)));                              
DDphi = -(2.*M1.*m2.*CosPhiTh - 2.*M2.*m2 - 2.*M2.*m1 - 2.*M2.*m2.*CosPhiTh + F1_y.*L.*m2.*CosPhi - 2.*F2_y.*L.*m1.*CosPhi - F2_y.*L.*m2.*CosPhi - F1_x.*L.*m2.*SinPhi + 2.*F2_x.*L.*m1.*SinPhi + F2_x.*L.*m2.*SinPhi + F1_y.*L.*m2.*cos(phi - 2.*th) + F2_y.*L.*m2.*cos(phi - 2.*th) + F1_x.*L.*m2.*sin(phi - 2.*th) + F2_x.*L.*m2.*sin(phi - 2.*th) + 2.*Dth.^2.*L.^2.*m2.^2.*SinPhiTh + L.*g.*m2.^2.*CosPhi - L.*g.*m2.^2.*cos(phi - 2.*th) + Dphi.^2.*L.^2.*m2.^2.*sin(2.*phi - 2.*th) + 2.*Dth.^2.*L.^2.*m1.*m2.*SinPhiTh + L.*g.*m1.*m2.*CosPhi - L.*g.*m1.*m2.*cos(phi - 2.*th))./(L.^2.*m2.*(2.*m1 + m2 - m2.*cos(2.*phi - 2.*th)));                            
                                                     
dX = [Dth; Dphi; DDth; DDphi];                      
                                                     
end                                                   
