% MAIN.m  --  Particle Swarm Optimization
%
% This script is used to run particle swarm optimization on several test
% functions. 
%

quadraticBowl = @(x)( sum(x.^2,1) );
quadraticBowlSkew2d = @(x)( 0.3*x(1,:).^2 + 4.2*x(2,:).^2 );
noisyBowl = @(x)( sum(x.^2 + 0.1*randn(size(x)),1) );

problem.options.populationCount = 15;   % Number of particles in the search
problem.options.w = 0.3;  % Particle damping coefficient
problem.options.pg = 0.7;   % Global search coefficient
problem.options.pl = 0.5;   % Local search coefficient
problem.options.maxIter = 20;  % Number of iterations

problem.xLow = -ones(2,1);
problem.xUpp = ones(2,1);

problem.objFun = quadraticBowl;
% problem.objFun = quadraticBowlSkew2d;
% problem.objFun = noisyBowl;

soln = particleSwarmOptimization(problem);

%%%% Plotting:
figure(1);
for i=1:problem.options.maxIter
    plotIterData(soln,i);   %Only works for 2D objective function
    pause(0.5);
end



