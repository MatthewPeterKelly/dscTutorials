function soln = smoothJointTrajectory(problem)


% Joint limits
qLow = problem.qLow;
qUpp = problem.qUpp;
dqMax = problem.dqMax;   %Joint speed limit

% Waypoints
tNode = problem.tNode;   %time
qNode = problem.qNode;   %angle

%Order of interpolating polynomial in each segment;
nGrid = problem.nGrid;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Build problem matricies                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

nDecVar = sum(nGrid);  %Number of decision variables
nSegment = length(nGrid);

% Rate constraint and objective matrix along each segment:
for i=1:length(nGrid)
    S(i) = chebyshevSegment(nGrid(i),tNode([i,i+1]),dqMax); %#ok<SAGROW>
end
A = structBlkDiag(S,'A');
H = structBlkDiag(S,'H');
D = structBlkDiag(S,'D');
DD = structBlkDiag(S,'DD');

b = []; t = [];  % Slow memory allocation - ok since few iterations
for i=1:length(nGrid)
    b = [b; S(i).b];  %#ok<AGROW>
    t = [t; S(i).t];  %#ok<AGROW>
end

%%%% Boundary Values:
nCstBc = 2*nSegment + 2*(nSegment+1);  %angle + rate

Aeq = zeros(nCstBc,nDecVar); beq = zeros(nCstBc,1);
finalIdx = cumsum(nGrid);
startIdx = 1 + [0, finalIdx(1:(end-1))];

cstIdx = 0;
for i=1:nSegment   %Angle at start of segment
    cstIdx = cstIdx + 1;
    Aeq(cstIdx,startIdx(i)) = 1;  beq(cstIdx) = qNode(i);
end
for i=1:nSegment   %Angle at end of segment
    cstIdx = cstIdx + 1;
    Aeq(cstIdx,finalIdx(i)) = 1;  beq(cstIdx) = qNode(i+1);
end

cstIdx = cstIdx + 1; Aeq(cstIdx,:) = D(1,:);  %zero initial velocity
cstIdx = cstIdx + 1; Aeq(cstIdx,:) = D(end,:);  %zero final velocity

% Defect constraint on rate at segment boundaries
for i=1:(nSegment-1)
    cstIdx = cstIdx + 1;
    Aeq(cstIdx,:) = D(startIdx(i+1),:) - D(finalIdx(i),:);
end

for i=1:(nSegment-1)
    cstIdx = cstIdx + 1;
    Aeq(cstIdx,:) = DD(startIdx(i+1),:) - DD(finalIdx(i),:);
end

%%%% Options:
options = optimset(...
    'Display','iter',...  % {'iter','final'}
    'Algorithm','interior-point-convex');

%%%% Build Problem:
problem.H = (H+H')/2;   %Correct for numerically introduced asymmetry
problem.f = zeros(nDecVar,1);
problem.Aineq = A;
problem.bineq = b;
problem.Aeq = Aeq;
problem.beq = beq;
problem.lb = qLow*ones(nDecVar,1);
problem.ub = qUpp*ones(nDecVar,1);
problem.x0 = [];
problem.options = options;
problem.solver = 'quadprog';


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                             Solve Problem                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

[x, fVal, exitFlag, output] = quadprog(problem);
for i=1:nSegment
    SS.grid.t = S(i).t';
    tSpan = SS.grid.t([1,end]);
    SS.grid.q = x(startIdx(i):finalIdx(i))';
    SS.grid.dq = chebyshevDerivative(SS.grid.q,tSpan);
    SS.interp.t = linspace( tSpan(1), tSpan(2), 5*length(SS.grid.q) );
    [q,dq,ddq] = chebyshevInterpolate(SS.grid.q,SS.interp.t,tSpan);
    SS.interp.q = q;
    SS.interp.dq = dq;
    SS.interp.ddq = ddq;
    soln.segment(i) = SS;
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                             Return Solution                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

soln.info = output;
soln.info.exitFlag = exitFlag;
soln.info.fVal = fVal;

soln.grid.t = t';
soln.grid.q = x';

end

