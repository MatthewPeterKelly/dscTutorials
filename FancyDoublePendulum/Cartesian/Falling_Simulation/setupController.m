function Control = setupController(P)

if P.Sim.springDoublePendulum

    Control.legOne.nomPos = 1; %(m)
    Control.legOne.Kp = 35;

    Control.legTwo.nomPos = 1; %(m)
    Control.legTwo.Kp = 35;

    %Then set everything to zero except for the proportional gains on the
    %leg length actuators. This turns them both into springs, which makes
    %the system conserve energy. This is a handle error check.

    %The Foot One actuator operates on the absolute angle of leg one
    Control.footOne.nomPos = 0;
    Control.footOne.nomVel = 0;
    Control.footOne.Kp = 0;
    Control.footOne.Kd = 0;

    %The Foot Two actuator operates on the absolute angle of leg two
    Control.footTwo.nomPos = 0;
    Control.footTwo.nomVel = 0;
    Control.footTwo.Kp = 0;
    Control.footTwo.Kd = 0;

    %The Leg One actuator operates on the length of leg one
    Control.legOne.nomVel = 0;
     Control.legOne.Kd = 0;

    %The Leg Two actuator operates on the length of leg two
    Control.legTwo.nomVel = 0;
    Control.legTwo.Kd = 0;

    %The Hip actuator operates on the angle between the legs: th2-th1
    Control.hip.nomPos = 0;
    Control.hip.nomVel = 0;
    Control.hip.Kp = 0;
    Control.hip.Kd = 0;
else
    %Allow system to be fully controlled. This will not necessarily
    %conserve energy.
    
    %The Foot One actuator operates on the absolute angle of leg one
    Control.footOne.nomPos = 0;
    Control.footOne.nomVel = 0;
    Control.footOne.Kp = 0;
    Control.footOne.Kd = 0;

    %The Foot Two actuator operates on the absolute angle of leg two
    Control.footTwo.nomPos = 0;
    Control.footTwo.nomVel = 0;
    Control.footTwo.Kp = 0;
    Control.footTwo.Kd = 0;

    %The Leg One actuator operates on the length of leg one
    Control.legOne.nomPos = 1; %(m)
    Control.legOne.nomVel = 0;
    Control.legOne.Kp = 100;
    Control.legOne.Kd = 50;

    %The Leg Two actuator operates on the length of leg two
    Control.legTwo.nomPos = 1; %(m)
    Control.legTwo.nomVel = 0;
    Control.legTwo.Kp = 100;
    Control.legTwo.Kd = 500;

    %The Hip actuator operates on the angle between the legs: th2-th1
    Control.hip.nomPos = 0;
    Control.hip.nomVel = 0;
    Control.hip.Kp = 0;
    Control.hip.Kd = 0;
end
    
end