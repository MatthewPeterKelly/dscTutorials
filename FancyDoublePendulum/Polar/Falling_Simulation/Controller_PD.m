function Actuators = Controller_PD(States,Control,Phase)

%This function implements a simple PD controller for every actuator.

%Unpack States into a struct
S = convert(States);   

%The Foot One actuator operates on the absolute angle of leg one
C = Control.footOne;
pos = S.th1;    
vel = S.dth1;
A.T1 = C.Kp*(C.nomPos-pos) + C.Kd*(C.nomVel-vel);
if strcmp(Phase,'F') || strcmp(Phase,'S2')
    A.T1 = 0*A.T1;    %Can't use ankle torques in mid-air!
end

%The Foot Two actuator operates on the absolute angle of leg two
C = Control.footTwo;
pos = S.th2;    
vel = S.dth2;
A.T2 = C.Kp*(C.nomPos-pos) + C.Kd*(C.nomVel-vel);
if strcmp(Phase,'F') || strcmp(Phase,'S1')
    A.T2 = 0*A.T2;    %Can't use ankle torques in mid-air!
end

%The Leg One actuator operates on the length of leg one
C = Control.legOne;
pos = S.L1;    
vel = S.dL1;
A.F1 = C.Kp*(C.nomPos-pos) + C.Kd*(C.nomVel-vel);

%The Leg Two actuator operates on the length of leg two
C = Control.legTwo;
pos = S.L2;    
vel = S.dL2;
A.F2 = C.Kp*(C.nomPos-pos) + C.Kd*(C.nomVel-vel);

%The Hip actuator operates on the angle between the legs: th2-th1
C = Control.hip;
pos = S.th2 - S.th1;    
vel = S.dth2 - S.dth2;
A.Thip = C.Kp*(C.nomPos-pos) + C.Kd*(C.nomVel-vel);

%Pack everything up and return:
Actuators = convert(A);

end