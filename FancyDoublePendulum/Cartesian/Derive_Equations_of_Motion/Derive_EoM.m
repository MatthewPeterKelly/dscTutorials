function Derive_EoM()
% Derive_EoM()
%
% This function generates the equations of motion for what I will call the 
% "Retractable Double Pendulum" model of walking. It uses the Matlab
% symbolic toolbox to generate the equations of motion, and then
% automatically writes them to a file. 
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
clc; clear; commandwindow;
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
% degrees of freedom in this model: position of foot one (x1,y1), position
% of the hip (x0,y0), and position of foot two (x2, y2).

% The position of the hip
x0 = sym('x0','real'); % Horizontal position of the hip
y0 = sym('y0','real'); % Vertical position of the hip

% The position of foot one
x1 = sym('x1','real'); % Horizontal position of foot one
y1 = sym('y1','real'); % Vertical position of foot one

% The position of foot two
x2 = sym('x2','real'); % Horizontal position of foot two
y2 = sym('y2','real'); % Vertical position of foot two

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

% Constraint force normal to each leg. Connects ankle torques to system
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
%                        State Derivatives                                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% The first time derivative of each state. These are considered known.
% The position of the hip
dx0 = sym('dx0','real'); % Horizontal velocity of the hip
dy0 = sym('dy0','real'); % Vertical velocity of the hip
dx1 = sym('dx1','real'); % Horizontal velocity of foot one
dy1 = sym('dy1','real'); % Vertical velocity of foot one
dx2 = sym('dx2','real'); % Horizontal velocity of foot two
dy2 = sym('dy2','real'); % Vertical velocity of foot two

% The second time derivative of each state. Goal is to find these.
ddx0 = sym('ddx0','real'); % Horizontal acceleration of the hip
ddy0 = sym('ddy0','real'); % Vertical acceleration of the hip
ddx1 = sym('ddx1','real'); % Horizontal acceleration of foot one
ddy1 = sym('ddy1','real'); % Vertical acceleration of foot one
ddx2 = sym('ddx2','real'); % Horizontal acceleration of foot two
ddy2 = sym('ddy2','real'); % Vertical acceleration of foot two


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Coordinate System & Kinematics                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Inertial reference frame:
i = [1;0;0];  %Positive horizontal axis
j = [0;1;0];  %Positive vertical axis
k = [0;0;1];  %Positive lateral axis

%Relative change in each ordinate along the legs
X1 = (x1-x0); dX1 = (dx1-dx0);
Y1 = (y1-y0); dY1 = (dy1-dy0);
X2 = (x2-x0); dX2 = (dx2-dx0);
Y2 = (y2-y0); dY2 = (dy2-dy0);

%length of both legs:
% This is a commonly used expression that is costly to calculate, so I
% numerically calculate it as an intermediate step.
L1 = sym('L1','real');
L2 = sym('L2','real');
dL1 = sym('dL1','real');
dL2 = sym('dL2','real');

%Angles of both legs, as measured in the k direction from the -j axis:
% th1 = sym('th1','real');
% th2 = sym('th2','real');
dth1 = sym('dth1','real');
dth2 = sym('dth2','real');

% Matlab symbolic toolbox was not being friendly, so I did a small amount
% of the math out by hand. I believe the following to be true:
% 
%  L^2 = x^2 + y^2
% (d/dt)(L) = (x*dx + y*dy)/L
% (d/dt)(atan2(x,y)) = (dx*y - x*dy)/L^2
%


Kinematics.L1 = sqrt(X1^2 + Y1^2);
Kinematics.L2 = sqrt(X2^2 + Y2^2);

Kinematics.dL1 = (X1*dX1 + Y1*dY1)/L1;
Kinematics.dL2 = (X2*dX2 + Y2*dY2)/L2;

Kinematics.th1 = atan2(X1,Y1);
Kinematics.th2 = atan2(X2,Y2);
Kinematics.dth1 = (dX1*Y1 - X1*dY1)/L1^2;
Kinematics.dth2 = (dX2*Y2 - X2*dY2)/L2^2;


% Unit vectors pointing from the hip to foot one (a1) and it's normal (b1)
a1 = (X1*i + Y1*j)/L1;
b1 = (-Y1*i + X1*j)/L1;

% Unit vectors pointing from the hip to foot two (a2) and it's normal (b2)
a2 = (X2*i + Y2*j)/L2;
b2 = (-Y2*i + X2*j)/L2;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Position Vectors                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Point 0 = Hip
% Point 1 = Foot One
% Point 2 = Foot Two

r0 = x0*i + y0*j;         % Position of Hip(Absolute)
r1 = x1*i + y1*j;         % Position of Foot One (Absolute)
r2 = x2*i + y2*j;         % Position of Foot Two (Absolute)


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Position Derivatives                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

dr0 = dx0*i + dy0*j;         % Velocity of Hip(Absolute)
dr1 = dx1*i + dy1*j;         % Velocity of Foot One (Absolute)
dr2 = dx2*i + dy2*j;         % Velocity of Foot Two (Absolute)

ddr0 = ddx0*i + ddy0*j;         % Acceleration of Hip(Absolute)
ddr1 = ddx1*i + ddy1*j;         % Acceleration of Foot One (Absolute)
ddr2 = ddx2*i + ddy2*j;         % Acceleration of Foot Two (Absolute)


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Linear Momentum Balance on Foot One                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

sum_of_forces = simplify(V1*j + H1*i + N1*b1 - m1*g*j + F1*a1);
linear_momentum_rate = m1*ddr1;

LMB_F1 = simplify(sum_of_forces - linear_momentum_rate);
LMB_F1_i = dot(LMB_F1,i);
LMB_F1_j = dot(LMB_F1,j);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Linear Momentum Balance on Foot Two                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

sum_of_forces = simplify(V2*j + H2*i + N2*b2 - m2*g*j + F2*a2);
linear_momentum_rate = m2*ddr2;

LMB_F2 = simplify(sum_of_forces - linear_momentum_rate);
LMB_F2_i = dot(LMB_F2,i);
LMB_F2_j = dot(LMB_F2,j);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Linear Momentum Balance on Hip                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

sum_of_forces = simplify(-F1*a1 - N1*b1 - F2*a2 - N2*b2 - M*g*j);
linear_momentum_rate = M*ddr0;

LMB_H = simplify(sum_of_forces - linear_momentum_rate);
LMB_H_i = dot(LMB_H,i);
LMB_H_j = dot(LMB_H,j);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Angular Momentum Balance on Leg One                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

sum_of_torques = (T1 + Thip - N1*L1)*k;
angular_momentum_rate = 0*k;   %Leg One has no mass

AMB_L1_k = dot(simplify(sum_of_torques - angular_momentum_rate),k);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Angular Momentum Balance on Leg One                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

sum_of_torques = (T2 - Thip - N2*L2)*k;
angular_momentum_rate = 0*k;   %Leg Two has no mass

AMB_L2_k = dot(simplify(sum_of_torques - angular_momentum_rate),k);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%      Collect Implicit Equations of Motion and Phase constraints         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% All phases of motion share these equations. An additional four equations
% are required to solve the system, which come from the 'definition' of
% each phase of motion.

Physics = [         LMB_F1_i;
                    LMB_F1_j;
                    LMB_F2_i;
                    LMB_F2_j;
                    LMB_H_i;
                    LMB_H_j;
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

Unknowns = [    ddx0;    ddy0;    
                ddx1;    ddy1;  
                ddx2;    ddy2;    
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
% %             ddx1==0;
% %             ddy1==0;
% %             H2==0;
% %             V2==0;  

Unknowns = [    ddx0;   ddy0;    
                H1;     V1  
                ddx2;   ddy2;    
                N1;     N2];      

disp(' -> Solving Single Stance One Dynamics')
Soln = jacobSolve(Physics,Unknowns);

Dyn.SingleOne = Soln;
Dyn.SingleOne.ddx1 = sym('0');
Dyn.SingleOne.ddy1 = sym('0');
Dyn.SingleOne.H2 = sym('0');
Dyn.SingleOne.V2 = sym('0');


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Solve Single Stance Two Dynamics                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% During single stance two, contact forces at Foot One are equal to zero. 
% Additionally, we know that the velocity and acceleration of Foot Two are
% both also equal to zero.
%
%             ddx2 == 0;   
%             ddy2 == 0;
%             H1==0;
%             V1==0;  

Unknowns = [    ddx0;   ddy0;    
                ddx1;   ddy1;  
                H2;     V2;    
                N1;     N2];     
                  
        
disp(' -> Solving Single Stance Two Dynamics')
Soln = jacobSolve(Physics,Unknowns);

Dyn.SingleTwo = Soln;
Dyn.SingleTwo.ddx2 = sym('0');
Dyn.SingleTwo.ddy2 = sym('0');
Dyn.SingleTwo.H1 = sym('0');
Dyn.SingleTwo.V1 = sym('0');



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                   Solve Double Stance Dynamics                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% During double stance, both feet remain stationary. 
% Additionally, we know that the velocity and acceleration of Foot One are
% both also equal to zero.
%               ddx1 == 0;
%               ddy1 == 0;
%               ddx2 == 0;
%               ddy2 == 0;

% In Double Stance, Both feet remain stationary. Add constraint equation.
% Note that the constraint is on acceleration, NOT velocity. This means
% that these equations will only yield the desired solutions if the initial
% velocity of Foot Two == 0;


Unknowns = [    ddx0;   ddy0  
                N1;     N2;     
                H1;     V1;     
                H2;     V2];    

disp(' -> Solving Double Stance Dynamics')
Soln = jacobSolve(Physics,Unknowns);

Dyn.Double = Soln;
Dyn.Double.ddx1 = sym('0');
Dyn.Double.ddy1 = sym('0');
Dyn.Double.ddx2 = sym('0');
Dyn.Double.ddy2 = sym('0');


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              Write Continuous Dynamics Function Files                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
disp(' -> Writing Continuous Dynamics Functions')

States = cell(12,2);
States(1,:) = {'x0','(m) Hip horizontal position'};
States(2,:) = {'y0','(m) Hip vertical position'};
States(3,:) = {'x1','(m) Foot One horizontal position'};
States(4,:) = {'y1','(m) Foot One vertical position'};
States(5,:) = {'x2','(m) Foot Two horizontal position'};
States(6,:) = {'y2','(m) Foot Two vertical position'};

States(7,:) = {'dx0','(m/s) Hip horizontal velocity'};
States(8,:) = {'dy0','(m/s) Hip vertical velocity'};
States(9,:) = {'dx1','(m/s) Foot One horizontal velocity'};
States(10,:) = {'dy1','(m/s) Foot One vertical velocity'};
States(11,:) = {'dx2','(m/s) Foot Two horizontal velocity'};
States(12,:) = {'dy2','(m/s) Foot Two vertical velocity'};

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

CommonExpressions = cell(2,2);
CommonExpressions(1,:) = {'L1s','L1.^2'};
CommonExpressions(2,:) = {'L2s','L2.^2'};

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

FileWritingSetup = Make_Struct(Dyn,States,Contacts,Actuators,...
    Kinematics,CommonExpressions,Parameters,FileData,Directory);

Write_ContinuousDynamics(FileWritingSetup);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Find System Energy                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

Energy.Potential.m1 = simplify(m1*g*dot(r1,j));
Energy.Potential.m2 = simplify(m2*g*dot(r2,j));
Energy.Potential.M = simplify(M*g*dot(r0,j));

Energy.Kinetic.m1 = simplify(0.5*m1*norm(dr1).^2);
Energy.Kinetic.m2 = simplify(0.5*m2*norm(dr2).^2);
Energy.Kinetic.M = simplify(0.5*M*norm(dr0)^2);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Write Energy Function                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
disp(' -> Writing Kinematics Function')

FileWritingSetup = Make_Struct(Energy,States,Parameters,Directory);

Write_Energy(FileWritingSetup);



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Write Power Function                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
disp(' -> Writing Actuator Power Function')

Power.legOne = F1*dL1;
Power.legTwo = F2*dL2;
Power.ankleOne = T1*dth1;
Power.ankleTwo = T2*dth2;
Power.hip = Thip*(dth2-dth1);

FileWritingSetup = Make_Struct(Power,Kinematics,...
    States,Actuators,Directory);

Write_ActuatorPower(FileWritingSetup);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Write Kinematics Function                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
disp(' -> Writing Kinematics Function')

FileWritingSetup = Make_Struct(Kinematics, States, Directory);

Write_Kinematics(FileWritingSetup);


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
%
% Since the masses in this system are essientally decoupled from eachother
% as far as impacts are concerned, the impact map can be simplified to the
% following statement: "The state is unchanged, except for the velocity of
% the mass that strikes the ground, which should go to zero".

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Write Impact Map Function Files                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
disp(' -> Writing Phase Map Function')

FileWritingSetup = ...
    Make_Struct(States, Directory);

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

A = simplify(jacobian(Equations,Unknowns));
b = simplify(Equations - A*Unknowns);
x = simplify(-A\b);

for i=1:length(Unknowns)
   Soln.(char(Unknowns(i))) = x(i); 
end

end
            


