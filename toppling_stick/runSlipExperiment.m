function Data = runSlipExperiment(setup,MoI,Mu)

%Get all of the parameters:
P.g = 1;
P.L = 1;  %Distance between end of stick and center of mass
P.FullLength = 2*P.L;   %Length of the entire stick
P.m = 1;
setup.P = P;

%Scale MoI
MoI = MoI*P.m*P.L^2;

%Store the independent parameters
nMoI = length(MoI);
nMu = length(Mu);

Data = cell(nMu,0);
for i = 1:nMu
    disp(['Running Mu = ' num2str(Mu(i))]);
    setup.P.u = Mu(i);
    Data(i).mu = Mu(i);
    Data(i).pos = zeros(nMoI,1);
    Data(i).neg = zeros(nMoI,1);
    Data(i).angle = zeros(nMoI,1);
    Data(i).moi = MoI;
    for j = 1:nMoI
        setup.P.I = MoI(j);
        setup.IC = getIC(setup.P,setup.perturbation);
        D = runSimulation(setup);
        if ~strcmp(D.phase{1},'HINGE')
            %Then the simulation started out sliding!
            Data(i).angle(j) = 0;
            Data(i).pos(j) = -1;
            Data(i).neg(j) = 0;
        else
            dist = getSlipDist(D);
            Data(i).pos(j) = sum(dist(dist<0));
            Data(i).neg(j) = sum(dist(dist>0));
            Data(i).angle(j) = D.raw(1).state.th(end);
        end
    end
end

end

function IC = getIC(P,delta)

%This function uses conservation of energy to pick an initial condition
%that has the same energy as the perfectly balanced inverted pendulum.

th = delta;
dth = topple_angularRate(th,P);

IC.th = -th;
IC.x = 0;
IC.y = 0;
IC.dth = -dth;
IC.dx = 0;
IC.dy = 0;

end