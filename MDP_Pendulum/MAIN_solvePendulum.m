% MAIN_solvePendulum.m
%
% This script finds the optimal policy for an inverted pendulum controller,
% by modeling it as a MDP and the solving with value iteration. The goal is
% to maintain upright balance, while minimizing actuator torque and
% preventing falls (nearing the edge of the boundry).
%
% NOTE: Most of this code is vectorized, but the barycentric interpolation
% methods are not. I strongly suggest using the mex versions of these
% functions (as implemented here). If you cannot compile the mex versions
% of these functions, then you can still make the code here work, by
% changing barycentric*_mex to barycentric* whenever it is called. See the
% script TEST_barycentric.m. for more details.
%

clc; clear;

% Physical constants for the pendulum
pendulum.dyn.m = 1;
pendulum.dyn.g = 9.81;
pendulum.dyn.c = 0.1;
pendulum.dyn.l = 0.5;
pendulum.dyn.timeConstant = sqrt(pendulum.dyn.l/pendulum.dyn.g);
pendulum.dyn.timeStep = 0.05*pendulum.dyn.timeConstant;

% Build the grid for the Markov Decision Process (MDP)
pendulum.grid.th.bnd = (pi/4)*[-1,1];  % ANGLE
pendulum.grid.th.nGrid = 45;
pendulum.grid.w.bnd = [-1,1];  %RATE
pendulum.grid.w.nGrid = 39;
gravityTorque = pendulum.dyn.m*pendulum.dyn.g*pendulum.dyn.l;
pendulum.grid.u.bnd = gravityTorque*[-1,1];  %TORQUE
pendulum.grid.u.nGrid = 25;

% Parameters for value iteration MDP solver
pendulum.opt.discount = 0.9;  %decay after one time constant
pendulum.opt.convergeTol = 1e-6;
pendulum.opt.maxIter = 1e4;
pendulum.opt.dispMod = 25;  

% Parameters for the cost function:
pendulum.opt.cost.angle = 100;  %Quadratic
pendulum.opt.cost.rate = 5; %Quadratic
pendulum.opt.cost.torque = 1; %Quadratic
pendulum.opt.cost.boundry.cost = 10;   %Penalty at the boundry
pendulum.opt.cost.boundry.width = 0.1;  %Boundry width (fraction)

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              Build MDP model of system and then solve                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
[V, P, S, A] = MDP_Pendulum(pendulum);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Plot the solution                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
soln.th = S(1,:);
soln.w = S(2,:);
soln.u = A(P);
soln.markerSize = 50*ones(size(soln.u));

figure(1); clf; colormap('jet')
scatter3(soln.th, soln.w, soln.u, soln.markerSize, soln.u, 'filled');
xlabel('angle (rad)')
ylabel('rate (rad/s)')
zlabel('torque (Nm)')
title('Optimal Policy')
view(2)

figure(2); clf; colormap('jet')
scatter3(soln.th, soln.w, V, soln.markerSize, V, 'filled');
xlabel('angle (rad)')
ylabel('rate (rad/s)')
zlabel('value ')
title('Value Function')
view(2)

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Run test simulations                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

figure(3); clf; hold on;
test.nSim = 100;
test.duration = 1.5;
test.nTime = ceil(test.duration/pendulum.dyn.timeStep);
test.t = linspace(0,test.duration,test.nTime);
test.th = pendulum.grid.th.bnd(1) + diff(pendulum.grid.th.bnd)*rand(1,test.nSim);
test.w = pendulum.grid.w.bnd(1) + diff(pendulum.grid.w.bnd)*rand(1,test.nSim);
test.sys = @(t,z) (pendulumSystem(z,A,P,pendulum.grid,pendulum.dyn));
test.z = rk4(test.sys,test.t,[test.th;test.w]);
for i=1:test.nSim
    thTest = reshape(test.z(1,i,:),test.nTime,1);
    wTest = reshape(test.z(2,i,:),test.nTime,1);
    u = pendulumController([thTest,wTest]',A,P,pendulum.grid);
    plot3(thTest, wTest, u, 'k-');
    plot3(thTest(1), wTest(1), u(1),'r.', 'MarkerSize',15);
    plot3(thTest(end), wTest(end), u(end), 'b.', 'MarkerSize',15);
end
drawBoundingBox(...
    pendulum.grid.th.bnd,pendulum.grid.w.bnd,pendulum.grid.u.bnd,...
    2,[0.1,0.9,0.1]);
xlabel('angle (rad)')
ylabel('rate (rad/s)')
zlabel('torque (Nm)')
title('Simulations')
view(2)





