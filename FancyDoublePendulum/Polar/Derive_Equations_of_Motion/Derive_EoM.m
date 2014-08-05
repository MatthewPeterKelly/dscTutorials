function Derive_EoM()
% Derive_EoM()
%
% This function generates the equations of motion for what I will call the 
% "Retractable Double Pendulum" model of walking. It uses the Matlab
% symbolic toolbox to generate the equations of motion, and then
% automatically writes them to a file. CodeGen is also used to create mex
% versions of the files, for faster run times.
%
% The model consists of a point mass at each foot and at the hip. The feet
% are connected to this hip by an extensible, actuated, leg. Each foot has
% an 'ankle actuator' to provide a control torque, and there is another
% torque actuator at the hip, connecting the two legs.
%
% Written by Matthew Kelly
% November 29, 2013
% Cornell University
%
% See also WRITE_CONTINUOUSDYNAMICS
clc; clear;
addpath ../Shared
Directory = '../computerGeneratedCode';  %Write all code in this directory
disp('Running Derive_EoM...')
disp(' -> Defining Model');
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                             Model                                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% This model of walking is comprised of three point masses: one at each
% foot and one at the hip. The equations are derived for three phases of
% motion: Flight, Single Stance, and Double Stance. The lets in this
% walking model are massless and have variable length. There are five
% controls available to the system: force actuator in each leg, foot torque
% when foot is in contact with the ground, and hip torque. There are six
% degrees of freedom in this model: position of foot one (x,y), absolute
% angle of each leg (th1,th2), and length of each leg (L1, L2).

% The position of the robot is based on foot one
x = sym('x','real'); % Horizontal position of foot one
y = sym('y','real'); % Vertical position of foot one

% Each leg of the robot has an absolute angle. In both cases this angle is
% measured from the positive horizontal axis (i direction)
th1 = sym('th1','real');    % Absolute angle of leg 1
th2 = sym('th2','real');  % Absolute angle of leg 2

% Each leg of the robot has a variable length:
L1 = sym('L1','real'); % Length of leg one, must be positive
L2 = sym('L2','real'); % Length of leg two, must be positive

% Each leg has a small mass at the foot and a large mass at the hip. Since
% the two hip masses are coincident, they are treated as a single mass. 
m1 = sym('m1','real');  % Mass of foot one
m2 = sym('m2','real');  % Mass of foot two
M = sym('M','real');  % Mass of the hip of the robot 

% The system experiences a constant gravitational acceleration:
g = sym('g','real');

% Axial Force along each leg. This force is considered to be an actuator 
% input to the system. Compression is positive.
F1 = sym('F1','real'); % Force in the stance leg
F2 = sym('F2','real'); % Force in the swing leg

% Constraint force at each joint. This force is always orthogonal to Fi
N1 = sym('N1','real'); % Constraint force at foot one
N2 = sym('N2','real'); % constraint force between legs

% There is a torque motor connecting the two legs, and an ankle motor on
% the stance leg.
T1 = sym('T1','real'); % Ankle Torque, acting on leg one
T2 = sym('T2','real'); % Ankle Torque, acting on leg two
Thip = sym('Thip','real'); % Hip torque. From leg one acting on leg two

% Ground contact forces at each foot
H1 = sym('H1','real'); % Horizontal contact force at foot one
V1 = sym('V1','real'); % Vertical contact force at foot one
H2 = sym('H2','real'); % Horizontal contact force at foot two
V2 = sym('V2','real'); % Vertical contact force at foot wto


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Coordinate System                                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Inertial reference frame:
i = [1;0;0];  %Positive horizontal axis
j = [0;1;0];  %Positive vertical axis
k = [0;0;1];  %Positive lateral axis

% Frame fixed to leg one:
a1 = cos(th1)*i + sin(th1)*j;  %Direction from foot one to hip
b1 = -sin(th1)*i + cos(th1)*j;  %Direction orthogonal to a1   

% Frame fixed to leg two:
a2 = cos(th2)*i + sin(th2)*j;  %Direction from hip to foot two
b2 = -sin(th2)*i + cos(th2)*j;  %Direction orthogonal to a2   


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Position Vectors                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Point 0 = Foot One
% Point 1 = Hip
% Point 2 = Foot Two

r0 = x*i + y*j;         % Position of Foot One (Absolute)

r10 = L1*a1;            % Position of hip with respect to the foot one:
r1 = r0 + r10;          % Position of the Hip (Absolute)

r21 = L2*a2;            % Position of foot two with respect to the hip:
r2 = r0 + r10 + r21;    % Position of Foot Two (Absolute)


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        State Derivatives                                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% The first time derivative of each state. These are considered known.
dx = sym('dx','real');
dy = sym('dy','real');
dth1 = sym('dth1','real');  
dth2 = sym('dth2','real'); 
dL1 = sym('dL1','real');
dL2 = sym('dL2','real');

% The second time derivative of each state. Goal is to find these.
ddx = sym('ddx','real');
ddy = sym('ddy','real');
ddth1 = sym('ddth1','real');  
ddth2 = sym('ddth2','real'); 
ddL1 = sym('ddL1','real');
ddL2 = sym('ddL2','real');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Frame Derivatives                                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%First time derivative of the stance frame
da1 = dth1*b1;
db1 = -dth1*a1;

%First time derivative of the swing frame
da2 = dth2*b2;
db2 = -dth2*a2;

%Second time derivative of the stance frame:
dda1 = simplify(ddth1*b1 - dth1^2*a1);
ddb1 = simplify(-ddth1*a1 - dth1^2*b1);

%Second time derivative of the swing frame:
dda2 = simplify(ddth2*b2 - dth2^2*a2);
ddb2 = simplify(-ddth2*a2 - dth2^2*b2);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Position Derivatives                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

dr0 = dx*i + dy*j;          % Velocity of Foot One (Absolute)

dr10 = dL1*a1 + L1*da1;     % Velocity of hip with respect to the foot one:
dr1 = dr0 + dr10;           % Velocity of the Hip (Absolute)

dr21 = dL2*a2 + L2*da2;     % Velocity of foot two with respect to the hip:
dr2 = dr0 + dr10 + dr21;    % Velocity of Foot Two (Absolute)


ddr0 = ddx*i + ddy*j;                      % Acc. of Foot One (Absolute)

ddr10 = ddL1*a1 + 2*dL1*da1 + L1*dda1;     % Acc. of hip wrt the foot one:
ddr10 = simplify(ddr10);
ddr1 = ddr0 + ddr10;                       % Acc. of the Hip (Absolute)

ddr21 = ddL2*a2 + 2*dL2*da2 + L2*dda2;     % Acc. of foot two wrt the hip:
ddr21 = simplify(ddr21);
ddr2 = ddr0 + ddr10 + ddr21;               % Acc. of Foot Two (Absolute)


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Linear Momentum Balance on Foot One                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

sum_of_forces = simplify(V1*j + H1*i - N1*b1 - m1*g*j - F1*a1);
linear_momentum_rate = m1*ddr0;

LMB_F1 = simplify(sum_of_forces - linear_momentum_rate);
LMB_F1_i = dot(LMB_F1,i);
LMB_F1_j = dot(LMB_F1,j);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Linear Momentum Balance on Leg One                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

sum_of_forces = simplify(N1*b1 + F1*a1 - F2*a2 - N2*b2 - M*g*j);
linear_momentum_rate = M*ddr1;

LMB_L1 = simplify(sum_of_forces - linear_momentum_rate);
LMB_L1_a1 = simplify(dot(LMB_L1,a1));
LMB_L1_b1 = simplify(dot(LMB_L1,b1));


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Linear Momentum Balance on Leg Two                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

sum_of_forces = simplify(V2*j + H2*i + N2*b2 + F2*a2 - m2*g*j);
linear_momentum_rate = m2*ddr2;

LMB_L2 = simplify(sum_of_forces - linear_momentum_rate);
LMB_L2_a2 = simplify(dot(LMB_L2,a2));
LMB_L2_b2 = simplify(dot(LMB_L2,b2));


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%             Angular Momentum Balance on Leg One about Hip               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

sum_of_moments = simplify(T1*k-Thip*k + cross(-L1*a1,N1*b1));
angular_momentum_rate = 0;  

AMB_L1 = sum_of_moments-angular_momentum_rate;
AMB_L1_k = dot(AMB_L1,k);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%           Angular Momentum Balance on Leg Two about Foot Two            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

sum_of_moments = simplify(Thip*k+T2*k + cross(-L2*a2,N2*b2));
angular_momentum_rate = 0;  

AMB_L2 = sum_of_moments-angular_momentum_rate;
AMB_L2_k = dot(AMB_L2,k);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%      Collect Implicit Equations of Motion and Phase constraints         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% All phases of motion share these equations. An additional four equations
% are required to solve the system, which come from the 'definition' of
% each phase of motion.

Physics = [         LMB_F1_i;
                    LMB_F1_j;
                    LMB_L1_a1;
                    LMB_L1_b1;
                    LMB_L2_a2;
                    LMB_L2_b2;
                    AMB_L1_k;
                    AMB_L2_k    ]; 


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Solve Flight Dynamics                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% During flight, the contact forces are all equal to zero. Treat as knowns.              
% %             H1==0;
% %             V1==0;
% %             H2==0;
% %             V2==0;   

Unknowns = [    ddx;    ddy;    
                ddth1;  ddth2;  
                ddL1;   ddL2;   
                N1;     N2];    

disp(' -> Solving Flight Dynamics')
Soln = jacobSolve(Physics,Unknowns);

Dyn.Flight = Soln;
Dyn.Flight.H1 = sym('0');
Dyn.Flight.V1 = sym('0');
Dyn.Flight.H2 = sym('0');
Dyn.Flight.V2 = sym('0');


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Solve Single Stance One Dynamics                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% During single stance one, contact forces at Foot Two are equal to zero. 
% Additionally, we know that the velocity and acceleration of Foot One are
% both also equal to zero.
% %             dx==0;   ddx==0;
% %             dy==0;   ddy==0;
% %             H2==0;
% %             V2==0;  

Unknowns = [    ddth1;  ddth2;  
                ddL1;   ddL2;   
                N1;     N2;     
                H1;     V1];    

disp(' -> Solving Single Stance One Dynamics')
Soln = jacobSolve(Physics,Unknowns);

Dyn.SingleOne = Soln;
Dyn.SingleOne.ddx = sym('0');
Dyn.SingleOne.ddy = sym('0');
Dyn.SingleOne.H2 = sym('0');
Dyn.SingleOne.V2 = sym('0');


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Solve Single Stance Two Dynamics                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% During single stance two, contact forces at Foot One are equal to zero. 
% Additionally, we know that the velocity and acceleration of Foot Two are
% both also equal to zero.
%
%             dr2 == 0;   ddr2 == 0;
%             H1==0;
%             V1==0;  

Unknowns = [    ddx;    ddy;    
                ddth1;  ddth2;  
                ddL1;   ddL2;   
                N1;     N2;     
                H2;     V2];    

%Constrain Foot Two to not accelerate
Foot_Cst = [    dot(ddr2,a2);
                dot(ddr2,b2)  ];            

Equations = [Physics; Foot_Cst];            
        
disp(' -> Solving Single Stance Two Dynamics')
Soln = jacobSolve(Equations,Unknowns);

Dyn.SingleTwo = Soln;
Dyn.SingleTwo.H1 = sym('0');
Dyn.SingleTwo.V1 = sym('0');



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                   Solve Double Stance Dynamics                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% During double stance, both feet remain stationary. 
% Additionally, we know that the velocity and acceleration of Foot One are
% both also equal to zero.
% %             dx==0;      ddx==0;
% %             dy==0;      ddy==0;
% %             dr2 == 0;   ddr2 == 0;

% In Double Stance, Both feet remain stationary. Add constraint equation.
% Note that the constraint is on acceleration, NOT velocity. This means
% that these equations will only yield the desired solutions if the initial
% velocity of Foot Two == 0;
Double_Cst = [      dot(ddr2,a2);
                    dot(ddr2,b2)  ];

Unknowns = [    ddth1;  ddth2;  
                ddL1;   ddL2;   
                N1;     N2;     
                H1;     V1;     
                H2;     V2];    
Equations = [Physics; Double_Cst];

disp(' -> Solving Double Stance Dynamics')
Soln = jacobSolve(Equations,Unknowns);

Dyn.Double = Soln;
Dyn.Double.ddx = sym('0');
Dyn.Double.ddy = sym('0');


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              Write Continuous Dynamics Function Files                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
disp(' -> Writing Continuous Dynamics Functions')

States = cell(12,2);
States(1,:) = {'x','(m) Foot One horizontal position'};
States(2,:) = {'y','(m) Foot One vertical position'};
States(3,:) = {'th1','(rad) Leg One absolute angle'};
States(4,:) = {'th2','(rad) Leg Two absolute angle'};
States(5,:) = {'L1','(m) Leg One length'};
States(6,:) = {'L2','(m) Leg Two length'};
States(7,:) = {'dx','(m/s) Foot One horizontal velocity'};
States(8,:) = {'dy','(m/s) Foot One vertical velocity'};
States(9,:) = {'dth1','(rad/s) Leg One absolute angular rate'};
States(10,:) = {'dth2','(rad/s) Leg Two absolute angular rate'};
States(11,:) = {'dL1','(m/s) Leg One extension rate'};
States(12,:) = {'dL2','(m/s) Leg Two extensioin rate'};

Contacts = cell(4,2);
Contacts(1,:) = {'H1','(N) Foot One, horizontal contact force'};
Contacts(2,:) = {'V1','(N) Foot One, vertical contact force'};
Contacts(3,:) = {'H2','(N) Foot Two, horizontal contact force'};
Contacts(4,:) = {'V2','(N) Foot Two, vertical contact force'};

Actuators = cell(5,2);
Actuators(1,:) = {'F1','(N) Compresive axial force in Leg One'};
Actuators(2,:) = {'F2','(N) Compresive axial force in Leg Two'};
Actuators(3,:) = {'T1','(Nm) External torque applied to Leg One'};
Actuators(4,:) = {'T2','(Nm) External torque applied to Leg Two'};
Actuators(5,:) = {'Thip','(Nm) Torque acting on Leg Two from Leg One'};

Parameters = cell(4,2);
Parameters(1,:) = {'m1','(kg) Foot One mass'};
Parameters(2,:) = {'m2','(kg) Foot Two mass'};
Parameters(3,:) = {'M','(kg) Hip mass'};
Parameters(4,:) = {'g','(m/s^2) Gravity'};

FileData = cell(3,4);
FileData(1,:) = {'Flight','dynamics_flight',...
    'Dymanics Model: retractable double pendulum biped',...
    'Motion Phase: Flight'};
FileData(2,:) = {'SingleOne','dynamics_singleStanceOne',...
    'Dymanics Model: retractable double pendulum biped',...
    'Motion Phase: Single Stance One'};
FileData(3,:) = {'SingleTwo','dynamics_singleStanceTwo',...
    'Dymanics Model: retractable double pendulum biped',...
    'Motion Phase: Single Stance Two'};
FileData(4,:) = {'Double','dynamics_doubleStance',...
    'Dymanics Model: retractable double pendulum biped',...
    'Motion Phase: Double Stance'};

FileWritingSetup = ...
    Make_Struct(Dyn,States,Contacts,Actuators,Parameters,FileData,Directory);

Write_ContinuousDynamics(FileWritingSetup);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Find System Energy                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

Energy.Potential.m1 = simplify(m1*g*dot(r0,j));
Energy.Potential.m2 = simplify(m2*g*dot(r2,j));
Energy.Potential.M = simplify(M*g*dot(r1,j));

Energy.Kinetic.m1 = simplify(0.5*m1*norm(dr0).^2);
Energy.Kinetic.m2 = simplify(0.5*m2*norm(dr2).^2);
Energy.Kinetic.M = simplify(0.5*M*norm(dr1)^2);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Write Kinematics Function                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
disp(' -> Writing Kinematics Function')

Kinematics = Make_Struct(r0,r1,r2,dr0,dr1,dr2);

FileWritingSetup = Make_Struct(Kinematics,Energy,States,Parameters,Directory);

Write_Kinematics(FileWritingSetup);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Write Power Function                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
disp(' -> Writing Actuator Power Function')

Power.legOne = F1*dL1;
Power.legTwo = F2*dL2;
Power.ankleOne = T1*dth1;
Power.ankleTwo = T2*dth2;
Power.hip = Thip*(dth2-dth1);

FileWritingSetup = Make_Struct(Power,States,Actuators,Directory);

Write_ActuatorPower(FileWritingSetup);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                             Impact Equations                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Assume that any transitions between phases can be modeled by external
% impluses being applied to either Foot One or Foot Two. 
%
% Each of the point masses in the model is connected to the others through
% force and torque actuators. Assume that at any instant in time there is a
% finite force or torque being provided by each of those actuators. In the
% limit as the duration of the impact goes to zero, the impulse transfered
% across those actuators also goes to zero.
%
% Thus, I conclude that the collision maps can be applied by solving the
% kinematic constraints before and after the collision.


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Velocity after collision                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% state derivatives after the collision
dx_plus = sym('dx_plus','real');
dy_plus = sym('dy_plus','real');
dth1_plus = sym('dth1_plus','real');  
dth2_plus = sym('dth2_plus','real'); 
dL1_plus = sym('dL1_plus','real');
dL2_plus = sym('dL2_plus','real');

%Frame derivatives after the collision
da1_plus = dth1_plus*b1;
da2_plus = dth2_plus*b2;

% position derivatives after the collision
dr0_plus = dx_plus*i + dy_plus*j;         
dr10_plus = dL1_plus*a1 + L1*da1_plus;     
dr1_plus = dr0_plus + dr10_plus;          
dr21_plus = dL2_plus*a2 + L2*da2_plus;    
dr2_plus = dr0_plus + dr10_plus + dr21_plus;  


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Impact Map                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Possible Phase Transitions:
%   Flight -> Single Stance One             F_S1
%   Flight -> Single Stance Two             F_S2
%   Double Stance -> Single Stance One      D_S1
%   Double Stance -> Single Stance Two      D_S2
%   Single Stance One -> Flight             S1_F
%   Single Stance One -> Double Stance      S1_D
%   Single Stance Two -> Flight             S2_F
%   Single Stance Two -> Double Stance      S2_D

% Since there are no impulsive connections between the point masses, the
% masses that are not directly involved in the collision will continue 
% moving without a change in their velocity.

% Although there are eight possible phase transitions, there are really
% only three different types of events:
%       Foot One hits the ground
%       Foot Two hits the ground
%       No collision (only release)

R0_Impact = dr0_plus;           %Stationary after collision
R2_Impact = dr2_plus;           %Stationary after collision
R0_Csv_Mtm = dr0_plus - dr0;    %Conserve Momentum (mass cancels)
R1_Csv_Mtm = dr1_plus - dr1;    %Conserve Momentum (mass cancels)
R2_Csv_Mtm = dr2_plus - dr2;    %Conserve Momentum (mass cancels)

ImpactOne = [R0_Impact;  R1_Csv_Mtm; R2_Csv_Mtm];
ImpactTwo = [R0_Csv_Mtm; R1_Csv_Mtm; R2_Impact];

Unknowns = [    dx_plus;    dy_plus;
                dth1_plus;  dth2_plus; 
                dL1_plus;   dL2_plus];

disp(' -> Solving Phase Maps')
SolnOne = jacobSolve(ImpactOne,Unknowns);
SolnTwo = jacobSolve(ImpactTwo,Unknowns);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Write Impact Map Function Files                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
disp(' -> Writing Phase Map Function')

FileWritingSetup = ...
    Make_Struct(SolnOne,SolnTwo,States,Parameters,Directory);

Write_PhaseMap(FileWritingSetup);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Write Conversion Function                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
disp(' -> Writing Conversion Function')

FileWritingSetup = Make_Struct(States,Actuators,Contacts,Directory);

Write_Convert(FileWritingSetup);

disp('DONE!');
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%                    SUB-FUNCTIONS                                  %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function Soln = jacobSolve(Equations,Unknowns)
%
% FUNCTION: 
%   Solve a nonlinear system of equations by assuming that it is linear in
%   the unknown variables (which is true for these mechanics problems)
%
% ARGUMENTS:
%   Equations = [Nx1] vector of symbolic expressions that are equal to zero
%   Unknowns = [Nx1] vector of symbolic variables to solve Equations for
%
% RETURNS:
%   Soln = a struct with a field for each unknown
%
% The matlab solve command seems to have a problem with solving large
% systems of non-linear equations. In the case of classical mechanics
% problems, it turns out that these systems are not too hard to solve
% because they are actually linear in the accelerations and constraint
% forces. Assuming that this is true, then you can transform the equations
% into a linear system by taking partial derivatives. Once this step is
% done, then matlab does a great job of solving the linear system.
%
% MATH:
%   Equations = 0;                  % By Definition
%   Equations = A*x + b;            % Assume: form, A independant* of x
%   A = jacobian(Equations wrt x);  % 
%   b = Equations - A*x;            %
%   0 = A*x + b;                    %
%   x = -A\b;                       % Solved!
%

A = jacobian(Equations,Unknowns);
b = Equations - A*Unknowns;
x = -A\b;

for i=1:length(Unknowns)
   Soln.(char(Unknowns(i))) = x(i); 
end

end
            


