function u = controller(q,dq,P)

%This is a simple controller to try for a stable orbit. It does not work
%very well... It is just hand tuned to get something reasonable...

r = q(1,:);
th1 = q(2,:);
th2 = q(3,:);
dr = dq(1,:);
dth1 = dq(2,:);
dth2 = dq(3,:);

escape = sqrt(2*P.G*P.mPlanet./r);
r = r/P.rPlanet;
dr = dr./escape;
phi = th2-th1;
dphi = dth2-dth1;

thrust = 0.03*(1.5-r) + 0.09*(0 - dr);
tip = 0.2*(pi/2-phi) + 0.2*(0-dphi);

u1 = thrust + tip;
u2 = thrust - tip;

u = [u1;u2];

end