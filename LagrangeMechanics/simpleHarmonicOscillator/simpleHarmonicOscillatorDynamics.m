function dz = simpleHarmonicOscillatorDynamics(~,z,k,m)

x = z(1,:);
v = z(2,:);

dx = v;
dv = -(k.*x)./m;

dz = [dx;dv];

end