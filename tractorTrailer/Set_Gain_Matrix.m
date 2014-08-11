function K = Set_Gain_Matrix(State, Input, Params)

%This function linearizes the plant about the current configuation and then
%uses a LQR to define the gain matrix.

%This model assumes that the only control input is the steering angle,
%because tractor trailer speed is set independently.

th = State(3);
phi = State(4);

v = Input(1);
psi = Input(2);

Lt = Params.Dyn.Lt;
Lc = Params.Dyn.Lc;

A = A_Matrix(phi,th,psi,v,Lt);
B = B_Matrix(phi,th,psi,v,Lt,Lc);

K = lqr(A,B,Params.Ctl.Q,Params.Ctl.R);

end