function u = control(z,k)

%PD controller with gravity compensation

x = z(1,:);  % angle
v = z(2,:);  % rate

%Unpack controller
kp = k(1,:);
kd = k(2,:);
kg = k(3,:);

u = kp.*x + kd.*v + kg.*sin(x);

end