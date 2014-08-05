function Actuators = Controller_PD(States,Control,Phase)

%This function implements a simple PD controller for every actuator.

%Unpack States into a struct, and then get angles and lengths
K = kinematics(States);

%The Foot One actuator operates on the absolute angle of leg one
C = Control.footOne;
pos = K.th1;    
vel = K.dth1;
A.T1 = C.Kp*(C.nomPos-pos) + C.Kd*(C.nomVel-vel);
if strcmp(Phase,'F') || strcmp(Phase,'S2')
    A.T1 = 0*A.T1;    %Can't use ankle torques in mid-air!
end

%The Foot Two actuator operates on the absolute angle of leg two
C = Control.footTwo;
pos = K.th2;    
vel = K.dth2;
A.T2 = C.Kp*(C.nomPos-pos) + C.Kd*(C.nomVel-vel);
if strcmp(Phase,'F') || strcmp(Phase,'S1')
    A.T2 = 0*A.T2;    %Can't use ankle torques in mid-air!
end

%The Leg One actuator operates on the length of leg one
C = Control.legOne;
pos = K.L1;    
vel = K.dL1;
A.F1 = C.Kp*(C.nomPos-pos) + C.Kd*(C.nomVel-vel);

%The Leg Two actuator operates on the length of leg two
C = Control.legTwo;
pos = K.L2;    
vel = K.dL2;
A.F2 = C.Kp*(C.nomPos-pos) + C.Kd*(C.nomVel-vel);

%The Hip actuator operates on the angle between the legs: th2-th1
C = Control.hip;
pos = K.th2 - K.th1;    
vel = K.dth2 - K.dth2;
A.Thip = C.Kp*(C.nomPos-pos) + C.Kd*(C.nomVel-vel);

%Pack everything up and return:
Actuators = convert(A);

end