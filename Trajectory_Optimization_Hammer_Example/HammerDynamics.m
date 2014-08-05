function dZ = HammerDynamics(~,Z,u,P)

%Let Th = 0 be straight up, Th = (pi/2) be striking the table
% Model the hammer as a simple point mass pendulum:
%
% Z is the modified state vector, where the cost function has been appended
% to the end of the normal state vector. Z = [Y;Cost];

g = P.dyn.gravity;  %gravity
L = P.dyn.length;  %length
m = P.dyn.mass;  %mass

I = m*L^2;   %Moment of inertia

Th = Z(1,:);
W = Z(2,:);

%Torque from gravity
Tau_Gravity = m*g*L*sin(Th);

%Rates:
dTh = W;
dW = (Tau_Gravity + u)/I;

%% Rate of change of cost function:

  switch P.cost.Method
    case 0 %Time Integral of Torque^2 
      %Cost = Integral of torque squared, so time-derivative is:
      dC = u.^2;
      
    case 1 %Time Integral of Torque*Rate
      %Cost = integral of (torque*rate) dt, so time-derivative is:
      beta = P.cost.Count_Negative_Work;   %How much to count negative work
      alpha = P.cost.Motor_Cost_Smoothing;    %Smoothing parameter (positive)
            
      dC = SmoothAbs(W.*u,alpha,beta,1);  %Smoothing and weighting
      
    otherwise
      error('Invalid Cost Method');
  end

  
%% Pack Up:
  dZ = [dTh;dW;dC];  %NOTE - dC is the derivative of the cost function
  
end

