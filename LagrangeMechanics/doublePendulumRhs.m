function dz = doublePendulumRhs(~,z,m1,m2,g,l1,I1,I2,d1,d2)

th1 = z(1,:);
w1 = z(2,:);
th2 = z(3,:);
w2 = z(4,:);

dth1 = w1;
dth2 = w2;
[dw1,dw2] = doublePendulumDynamics(th1,th2,w1,w2,m1,m2,g,l1,I1,I2,d1,d2);

dz = [dth1;dw1;dth2;dw2];

end