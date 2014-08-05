function Xd = Pendulum_Cart_Dynamics_Forced(X,F,P_Dyn)
% Xd = Pendulum_Cart_Dynamics_Forced(X,F,P_Dyn)
%
% FUNCTION:
%   This function is used to compute the dynamics of the pendulum cart
%   system. The inputs are the system state, actuator force, and paramters.
%   
% INPUTS:
%
%   X = [4 x N] state matrix: [x;v;th;w]
%   F = [1 x N] actuator force (applied horizontally to cart)
%   P_Dyn = a parameter struct with fields:
%       M = cart mass
%       m = bob mass
%       L = pendulum length
%       g = gravitational acceleration
%
% OUTPUTS:
%   dX = [4 x N] derivative of the state matrix [dx;dv;dth;dw]
%
% NOTES:
%   The equations of motion are found using a cartesian force balance for
%   each rigid body
%
%   The cart is modelled as a point mass, and the pendulum is modelled as a
%   point mass at the end of a slender rod. There is no friction in the
%   system. 
%
%   Parts of the derivation are shown below in comments. This code has been
%   partially optimized, so some of the comments show intermediate steps
%   that were bypassed.
%

%Define Parameters
m = P_Dyn.m;    %Pendulum point mass
M = P_Dyn.M;    %Cart mass
L = P_Dyn.L;    %Pendulum length (to point mass)
g = P_Dyn.g;    %Gravity

%Define input states:
% % x = X(1);    %Horizontal position of the cart
v = X(2,:);      %Horizontal velocity of the cart
th = X(3,:);     %Angle of the pendulum: 0 = straight up, pi = straight down
w = X(4,:);      %Angular rate of the pendulum

%Evaluate sines and cosines
Sin = -sin(th);
Cos = -cos(th);
One = ones(size(Sin));

%"Constant terms"
% % App_Forces = [...
% %     -F;
% %     M*g;
% %     -m*L*w^2*Sin;
% %     m*g + m*L*w^2*Cos];

% % %Matrix for of coeffieients: 
% %     % Coeff * [Tension; Normal; Trans. Acc; Rot Acc] =
% %     % Applied_Forces( X0, Y0, X1, Y1 )
% % Coeff = [...
% %     sin(th)     0   -M  0;
% %     -cos(th)    1   0   0;
% %     -sin(th)    0   -m  -m*L*cos(th);
% %     cos(th)     0   0   -m*L*sin(th)];

% Analytic Inverse of Coeff via Mathematica:
Det = m*L*(M*Cos.^2 + (m+M)*Sin.^2);
inv_Det = 1./Det; 
% % Adj = [...
% %     m^2*L*Sin       0   -m*M*L*Sin          m*M*L*Cos;
% %     m^2*L*Sin*Cos   1   -m*M*L*Sin*Cos      m*M*L*Cos^2;
% %     -m*L          	0   -m*L*Sin^2          m*L*Cos*Sin;
% %     m*Cos        	0   -M*Cos              -(M+m)*Sin];

% % Inv = inv_Det*Adj;
% % UnKnowns = Inv*App_Forces; %Solve linear system of equations

% % % % Tension = Unknowns(1);
% % % % Normal = UnKnowns(2);
% % vd = UnKnowns(3);
% % wd = UnKnowns(4);

%Unknowns (By Hand)
% % UnKnowns = inv_Det*[...
% %     %-F*               M*g*       -m*L*w^2*Sin*         (m*g + m*L*w^2*Cos)*
% %     -F*m^2*L*Sin     + 0   +  m*L*w^2*Sin*m*M*L*Sin     + (m*g + m*L*w^2*Cos)*m*M*L*Cos ;    %Tension
% %     -F*m^2*L*Sin*Cos + M*g +  m*L*w^2*Sin*m*M*L*Sin*Cos + (m*g + m*L*w^2*Cos)*m*M*L*Cos^2 ;  %Normal Force
% %     F*m*L            + 0   +  m*L*w^2*Sin*m*L*Sin^2     + (m*g + m*L*w^2*Cos)*m*L*Cos*Sin ;  %Translational acceleration
% %     -F*m*Cos         + 0   +  m*L*w^2*Sin*M*Cos         +  -(m*g + m*L*w^2*Cos)*(M+m)*Sin ]; %Angular Acceleration

%Broken Out UnKnowns (By Hand)
vd = (F*m*L  + 0 +  m*L*w.^2.*Sin*m*L.*Sin.^2 + (m*g*One + m*L*w.^2.*Cos).*m*L.*Cos.*Sin).*inv_Det;%Translational acceleration
wd = (-F.*m.*Cos  + 0  + m*L*w.^2.*Sin.*M.*Cos  +  -(m*g*One + m*L*w.^2.*Cos)*(M+m).*Sin).*inv_Det;%Angular Acceleration

%Defined derivatives:
xd = v;
thd = w;

%Express as a vector
Xd = [xd;vd;thd;wd];

end
