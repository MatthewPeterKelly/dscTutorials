function DeriveDynamicsDP(P)

%FUNCTION:
%   This function is used to derive the analytic equations of motion for a
%   forced double pendulum. This also does the kimenatics and energy
%   calculations. 


%State Naming Conventions:
% th = absolute angle of the first link measured wrt the positive horizontal axis (i)
% phi = absolute angle of the second link measured wrt the positive horizontal axis (i)
% Dth = first time derivative of th
% Dphi = first time derivative of phi

%NOTE: It might be faster to use the jacobian command to find the symbolic
%coeffieints of the mass matrix and then invert the system, rather than
%symbolically inverting the matrix.
%

% Define the angles and derivatives
    th = sym('th');    
    Dth = sym('Dth');
    DDth = sym('DDth');

    phi = sym('phi');
    Dphi = sym('Dphi');
    DDphi = sym('DDphi');
            
% Define the scalar parameters
    g = sym('g');   %gravity
    L = sym('L');    %Lenght of each link
    m1 = sym('m1');    %mass of the link
    m2 = sym('m2');    %mass of the link
    
% Known Vectors
    F1_x = sym('F1_x');
    F1_y = sym('F1_y');
    F1 = [F1_x;F1_y];   %External force at CoM of first link

    F2_x = sym('F2_x');
    F2_y = sym('F2_y');
    F2 = [F2_x;F2_y];   %External force at CoM of 2nd link

    M1 = sym('M1');   %Input torque on link on
    M2 = sym('M2');   %Input torque between links 1 and 2

% UNIT VECTORS:
    i = [1;0];
    j = [0;1];

    a1 = cos(th)*i + sin(th)*j;   %The direction along the first link 
    b1 = -sin(th)*i + cos(th)*j;
    Da1 = Dth*b1;
    Db1 = -Dth*a1;
    DDa1 = DDth*b1 + Dth*Db1;
    DDb1 = -DDth*a1 -Dth*Da1;

    a2 = cos(phi)*i + sin(phi)*j;   %The direction along the second link 
    b2 = -sin(phi)*i + cos(phi)*j;
    Da2 = Dphi*b2;
    Db2 = -Dphi*a2;
    DDa2 = DDphi*b2 + Dphi*Db2;
    DDb2 = -DDphi*a2 -Dphi*Da2;
    
%Weights
    W1 = -m1*g*j;   %weight of the first link
    W2 = -m2*g*j;   %weight of the second link

%Kinematic vectors       
    r_P1_O = L*a1;    %Position of the point mass 1
    v_P1_O = L*Da1;
    a_P1_O = L*DDa1;

    r_P2_O = L*a1 + L*a2;    %Position of point mass 2
    v_P2_O = L*Da1 + L*Da2;
    a_P2_O = L*DDa1 + L*DDa2;
       
    r_P2_P1 = L*a2;   %Relative position of the tip with respect to the mid joint
    
    
%Momentum balance for both links about the origin
    %       = Moment from weight 1 +  moment from weight 2 + Control torque 1
    Sum_M_O = Cross(r_P1_O,F1+W1) + Cross(r_P2_O,F2+W2) + M1;
    %        = translation mass 1  + translation mass 2
    Sum_DH_O = Cross(r_P1_O,m1*a_P1_O) + Cross(r_P2_O,m2*a_P2_O);

%Momentum balance for second link about central pivot
    %         moment from weight 2  +  control moment
    Sum_M_P1 = Cross(r_P2_P1,F2+W2) + M2;
    %         translation mass 2 + 
    Sum_DH_P1 = Cross(r_P2_P1,m2*a_P2_O);

%Simplify
    AMB_1 = simplify(Sum_M_O - Sum_DH_O);
    AMB_2 = simplify(Sum_M_P1 - Sum_DH_P1);

% Solve the system of equations:
    Soln = solve( AMB_1, ...    %AMB for system
           AMB_2, ...    %AMB for second link
            DDth,...                      %acceleration of first joint
            DDphi  );                    %acceleration of second joint

    Soln.DDth = simplify(Soln.DDth);   %Simplify the result    
    Soln.DDphi = simplify(Soln.DDphi);   %Simplify the result    

%Write the results to a file  --  these are scripts!    
if P.misc.CppFlag
    WriteDynamicsFile_Cpp    %Call a script that write the dynamics to a file
else
    WriteDynamicsFile_Matlab;   %Call a script that write the dynamics to a file
end
WriteKinematicsFile_Matlab;   %Call a script that writes the equations for kinetics


    
end