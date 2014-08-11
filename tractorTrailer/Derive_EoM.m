function [dStates, A, B, Positions] = Derive_EoM()

%This function derives the equations of motion for a tractor trailer truck.

%The outputs assume that the states and inputs are being listed in the
%order in which they are defined

%% Parameters

Lt = sym('Lt');
Lc = sym('Lc');

%% Inputs

v = sym('v');
psi = sym('psi');

%% States

x = sym('x');
y = sym('y');
th = sym('th');
phi = sym('phi');

%% State Derivatives

Dx = sym('Dx');
Dy = sym('Dy');
Dth = sym('Dth');
Dphi = sym('Dphi');

%% Equations of motion

%Unit Vectors
    i = [1;0];
    j = [0;1];

    An = cos(th)*i + sin(th)*j;
    At = -sin(th)*i + cos(th)*j;
    Bn = cos(phi)*An + sin(phi)*At;
    Bt = -sin(phi)*An + cos(phi)*At;
    Cn = cos(psi)*Bn + sin(psi)*Bt;
    Ct = -sin(psi)*Bn + cos(psi)*Bt;

%Derivatives of Unit Vectors
    dAn = Dth*At;
    dAt = -Dth*An;
    dBn = (Dphi + Dth)*Bt;
    dBt = -(Dphi + Dth)*Bn;

%Position Vectors
    Pa = x*i + y*j;
    Pb = Pa + Lt*At;
    Pc = Pb + Lc*Bt;
    Positions = [Pa,Pb,Pc];
    
%Derivatives of Position Vectors
    dPa = Dx*i + Dy*j;
    dPb = dPa + Lt*dAt;
    dPc = dPb + Lc*dBt;
    
%Trailer Back Wheel Constraint
    RHS_1 = 0;
    LHS_1 = Dot(dPa,An);
    
%Cab Back Wheel Constraint
    RHS_2 = 0;
    LHS_2 = Dot(dPb,Bn);
    
%Cab Front Wheel Constraints
    RHS_3 = dPc(1);
    LHS_3 = v*Ct(1);
    
    RHS_4 = dPc(2);
    LHS_4 = v*Ct(2);

%Solve for State Derivatives
    Solution = solve(...
        RHS_1-LHS_1, RHS_2-LHS_2, RHS_3-LHS_3, RHS_4-LHS_4,...
        Dx, Dy, Dth, Dphi); 
    
%Express as Vector [Dx, Dy, Dth, Dphi]

    dStates = [...
        Solution.Dx;
        Solution.Dy;
        Solution.Dth;
        Solution.Dphi];
    
%% Jacobian Calculations

    J_x = diff(dStates,x);
    J_y = diff(dStates,y);
    J_th = diff(dStates,th);
    J_phi = diff(dStates,phi);
    J_v = diff(dStates,v);
    J_psi = diff(dStates,psi);
    
    A = [J_x, J_y, J_th, J_phi];
    B = [J_v, J_psi];

end