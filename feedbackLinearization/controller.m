function u = controller(z,dyn,ref)

q1 = z(1);
q2 = z(2);
dq1 = z(3);
dq2 = z(4);

q = [q1;q2];
dq = [dq1;dq2];

% Compute reference trajectories:
p = ref.c*q;
dp = ref.c*dq;
hRef = ppval(ref.pp.h, p);
dhRef = ppval(ref.pp.dh, p);
ddhRef = ppval(ref.pp.ddh, p);
h = ref.H*q;
dh = ref.H*dq./dp;  %Chain rule again to make in terms of phase

% Linear controller:
kp = ref.wn^2;
kd = 2*ref.xi*ref.wn;
v = kp*(h-hRef) + kd*(dh-dhRef);   %%%% HACK

v = -v;  %%%% HACK

% D*ddq + G = B*u
[D,G,B] = autoGen_acrobotDynamics(...
    q1,q2,dq1,dq2,...
    dyn.m1,dyn.m2,dyn.g,dyn.l1,dyn.l2);

u = controlMap(dq, v, dhRef, ddhRef, ref.c, ref.H, D, B, G);

end