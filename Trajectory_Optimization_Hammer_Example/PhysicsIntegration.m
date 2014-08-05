function Soln = PhysicsIntegration(x,P)

%This function is used to run the physics for the system. Since it is
%called from both NonLinCon and Objective_Function, I use a persistent
%variable to prevent it from doing the same integration twice. Soln
%contains a field for the defect vector and another field for the
%value of the cost function.

persistent xLast;   %store the last input x
persistent SolnLast;   %store the last solution

%CALLNUM is used to keep track of how many times this function has been
%called. This is then used to decide if a plot should be generated.
persistent CALLNUM; if isempty(CALLNUM), CALLNUM = 0; end; %Initialize

%See if the answer to this problem has already been found:
if isequal(x,xLast)
  %Then we already solved this problem - return the previous solution
  Soln = SolnLast;  
else
  %This is a new problem, create a new solution
  xLast = x;
  X = Convert_State(x,P.structDat);
  CALLNUM = CALLNUM + 1;  %Increment


    %Break up the data into blocks:

      nGridPts = P.nGridPts;
      IdxLow = 1:(nGridPts-1);
      IdxUpp = 2:nGridPts;

      yLow = X.state(:,IdxLow);
      yUpp = X.state(:,IdxUpp);

      uLow = X.torque(IdxLow);
      uUpp = X.torque(IdxUpp);
  
  %Since we are using a piecewise linear cost function, the value of the
  %cost function a the middle of a time step is the mean of the two end
  %points. If we were using a nonlinear fitting function, then we would
  %need to evaluate u(tMid), where t is the time vector.
  uMid = 0.5*(uLow+uUpp);   

  %We need to use the integration algorithm to integrate the cost function 
  %for us. This is awkward, because we don't want fmincon to treat the cost
  %like a normal state, because then we add nGridPts extra constraints and
  %states for the algorithm to deal with. This increases complexity and can
  %cause convergence to fail. The solution is to append a cost state to the
  %full state before passing it to the integration algorithm. Let's call the
  %new state z.

  dummyCost = zeros(1,(nGridPts-1));
  zLow = [yLow;dummyCost];

%Compute a 4th order runge Kutta integration step. A fixed-step method is
%preferable over a variable-step method because the fixed-step method
%provides more consistent results, which is important for convergence of the
%optimization method.

  dynFunc = P.dyn.UserFunc;   %Get the handle for the dynamics function
  
  h = X.duration/(nGridPts-1);   %Divide by number of STEPS, not GRIDPOINTS
  k1 = feval(dynFunc, zLow, uLow);
  k2 = feval(dynFunc, zLow+0.5*h*k1, uMid);
  k3 = feval(dynFunc, zLow+0.5*h*k2, uMid);
  k4 = feval(dynFunc, zLow+h*k3, uUpp);
  zUpp_RK4 = zLow + h*(1/6)*(k1 + 2*k2 + 2*k3 + k4);
  
  %Break apart the robot state and the cost state
  yUpp_RK4 = zUpp_RK4(1:(end-1),:);   %Get the original full state
  Cost_Segments = zUpp_RK4(end,:);   %Get the cost segments
  
  Total_Cost = sum(Cost_Segments);   %Get the total cost
  
%Compute the discrete dynamics:
  %NOTE - this map will occur whether or not the state is on the impact
  %surface. We will need to constrain this to be true. This constraint is
  %enforced in the bounds of the problem.
  yMinus = X.state(:,end);  %The last state is when the impact occurs 
  yPlus = HammerImpact(yMinus,P);  %State after the impact
  yStart = X.state(:,1);  %The state at the beginning of the trajectory
  
%Now compute the defects:
  Defect_Continuous = yUpp(1:2,:)-yUpp_RK4;  %This makes the trajectory continuous
  Defect_Discrete = yStart - yPlus;  %This is the periodic constrain 
  
  %Combine defects and flatten to a vector:
  Defects = [Defect_Continuous, Defect_Discrete];  %Pack up to return
  [N,M] = size(Defects);
  Defect_Vec = reshape(Defects,N*M,1);  %Collapse to a vector

  %Pack things up:
  Soln.Total_Cost = Total_Cost;
  Soln.Defect_Vec = Defect_Vec;
  Soln.Cost_Segments = Cost_Segments;
  
  %Store as persistent
  SolnLast = Soln;
  
  %Decide whether or not to show a plot:
  if P.disp.intermediatePlot, Plotting_Script, end

end
end