function [zSoln, zGrid] = chebyshevODEsolve(Input)
%zSoln = chebyshevODEsolve(Input)
%
% FUNCTION:  
%   chebyshevODEsolve solves the given ordinary differential equation at
%   the chebyshev nodes of a given order. This method is preffered to a
%   solver like ode45 for high-accuracy solutions, assuming that a
%   sufficiaently large order polynomial is used.
%
% INPUTS:
%   Input = a struct with a few fields describing the ode:
%       order = non-negative integer, giving order of the interpolant
%       tspan = [1 x 2] time span for solution: [start time, end time]
%       z0 = [N x 1] initial state for the system
%       userFunc = function handle for the system dynamics. Form:
%                dz = f(t,z)
%
% OUTPUTS:
%   zSoln = [N x (order+1)] the solution to the ode at each of the
%       chebyshev grid points. 
%   zGrid = [1 x (order+1)] the vector of times for the chebyshev grid
%       points
%
% NOTES:
%   In order to get the value of the solution at an arbitrary time vector t
%   use the following command:
%       y = chebyshevInterpolate(zSoln,t,[min(zGrid), max(zGrid)]);
%
%   The accuracy of the solution is highly dependant on the order of the
%   underlying chebyshev polynomial. More specifically, the accuracy will
%   be close to machine precision only for polynomials above some order.
%   Make sure that this is the case by doing a convergence test. The
%   solution should change by less than 1e-12 between solutions if the
%   order is sufficiantly large.
%

%Unpack the inputs:
order = Input.order;
tspan = Input.tspan;
z0 = Input.z0;
userFunc = Input.userFunc;
n = order+1;

%Get the differentiation matrix. Note that this is constant
D = chebyshevDifferentiationMatrix(n,tspan);

%Set up the chebyshev grid and initialization
time = chebyshevPoints(n,tspan);
zSubGuess = zeros(2*(n-1),1);

%Set up the problem for sending to fsolve
Problem.x0 = zSubGuess;
Problem.objective = @(zSub)defectFunc(time,z0,zSub,D,userFunc);
Problem.options = optimset(...
    'Algorithm','Levenberg-Marquardt',...
    'TolFun',1e-14,...
    'TolX',1e-14,...
    'Display','off');
Problem.solver = 'fsolve';

%Use fsolve to find the best set of chebyshev values
zSubSoln = fsolve(Problem);

%Reshape the solution and include the initial condition
zSubM = reshape(zSubSoln,2,(n-1));
zSoln = [z0,zSubM];

%Get the time for each of the chebyshev nodes
zGrid = time;

end

%
%%%% SUB FUNCTIONS %%%%
%

function ZERO = defectFunc(t,z0,zSub,D,userFunc)

%This function is used in the inner loop of fsolve. It checks how well the
%current guess fits the underlying differential equations.

%Unpack inputs
N = length(zSub);
zSubM = reshape(zSub,2,N/2);
z = [z0,zSubM];

%Get the derivatives as predicted by the user-provided function
dz_dynamics = feval(userFunc,t,z);

%Get the derivatives as predicted by differentiating the solution directly
dz_chebyshev = (D*z')';

%The solution is found as norm(defects) -> 0
defects = dz_dynamics - dz_chebyshev;
[a,b] = size(defects);

%Reformat for sending back to fsolve.
ZERO = reshape(defects,a*b,1);


end

