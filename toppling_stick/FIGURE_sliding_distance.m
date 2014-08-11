%FIGURE_sliding_distance

%%%% EXPERIMENT %%%%
%
% For a slendar rod toppling from rest, plot how the slip distance varies
% as a function of coefficient of friction.
%
%

clc; clear;

setup.Tspan = [0,10];  %only used for timeout of the simulation
setup.tol = 1e-12;   %Accuracy of the intergation method
setup.dataFreq = 750;   %How much data to return?
setup.solver = @ode45;   %Tell the simulation to use a still solver.

%%%% HACK %%%%
% If the perturbation is too small, then numerical errors start to creep
% into the results. If perturbation is larger than the critical angle of
% the smallest non-zero coefficient of friction, then it will cause a
% direct error by violating the initial phase assumption.
setup.perturbation = 1e-2;
%%%% DONE %%%%

MoI = logspace(-3,0,250);
Mu = [0,0.05,0.1,0.5,1,5,10,inf];
P.m = 1;
P.g = 1;
P.L = 1;

%%%% Run big experiment %%%%
Data = runSlipExperiment(setup,MoI,Mu);

%%%% Plotting %%%%
N_mu = length(Data);
xBnd = [min(Data(1).moi), max(Data(1).moi)];
Mu = zeros(N_mu,1);
style = {'k--','b--','r--','m--','g--',...
    'k-','b-','r-','m-','g-'...
    'k:','b:','r:','m:','g:'};
LINEWIDTH = 3;
FontSize.Title = 16;
FontSize.label = 12;
names = cell(1,N_mu);
for i=1:N_mu
    Mu(i) = Data(i).mu;
    names{i} = num2str(['u = ' num2str(Mu(i))]);
end

%%%% Distance %%%%
H_dist = figure(100); clf; hold on;
set(H_dist,'Name','SlipDist','NumberTitle','off')
IDX = false(2,length(Mu));
for i=1:length(Data)
    sty = style{mod(i-1,length(style))+1};
    subplot(2,1,1); hold on;
    idx = Data(i).pos~=0;  IDX(1,i) = sum(idx)~=0;
    semilogx(Data(i).moi(idx),Data(i).pos(idx),sty,'LineWidth',LINEWIDTH);
    title('Backwards Slip (falling forward)','FontSize',FontSize.Title)
    xlabel('Moment of Inertia','FontSize',FontSize.label)
    ylabel('Slip Distance','FontSize',FontSize.label)
    set(gca,'Xscale','log')
    subplot(2,1,2); hold on;
    idx = Data(i).neg~=0;  IDX(2,i) = sum(idx)~=0;
    semilogx(Data(i).moi(idx),Data(i).neg(idx),sty,'LineWidth',LINEWIDTH);
    title('Forwards Slip (falling forward)','FontSize',FontSize.Title)
    xlabel('Moment of Inertia','FontSize',FontSize.label)
    ylabel('Slip Distance','FontSize',FontSize.label)
    set(gca,'Xscale','log')
end
%subplot(2,1,1); legend(names(IDX(1,:)),'Location','NorthEast');
% extents = [xBnd,-1,0];  axis(extents);
subplot(2,1,2); legend(names(IDX(2,:)),'Location','SouthWest');
% extents = [xBnd,0,0.14];

%%%% Critical Angle %%%%
H_angle = figure(101); clf; hold on;
set(H_angle,'Name','CriticalAngle','NumberTitle','off')
IDX = false(2,length(Mu));
for i=1:length(Data)
    sty = style{mod(i-1,length(style))+1};
    
    idx1 = Data(i).pos~=0;  IDX(1,i) = sum(idx1)~=0;
    subplot(2,1,1); hold on
    semilogx(Data(i).moi(idx1),-Data(i).angle(idx1)*180/pi,sty,'LineWidth',LINEWIDTH);
    title('Backwards Slip (falling forward)','FontSize',FontSize.Title)
    xlabel('Moment of Inertia','FontSize',FontSize.label)
    ylabel('Critical Angle (deg)','FontSize',FontSize.label)
    set(gca,'Xscale','log')
    
    idx2 = Data(i).neg~=0; IDX(2,i) = sum(idx2)~=0;
    idx2a = idx2 & idx1;   %In these special cases, it slips backwards and then forwards. Only plot the FIRST critical angle.
    idx2b = idx2 & ~idx1;
    subplot(2,1,2); hold on;
%     semilogx(Data(i).moi(idx2a),-Data(i).angle(idx2a)*180/pi,sty,'LineWidth',LINEWIDTH);
    semilogx(Data(i).moi(idx2b),-Data(i).angle(idx2b)*180/pi,sty,'LineWidth',LINEWIDTH);
    title('Forwards Slip (falling forward)','FontSize',FontSize.Title)
    xlabel('Moment of Inertia','FontSize',FontSize.label)
    ylabel('Critical Angle (deg)','FontSize',FontSize.label)
    set(gca,'Xscale','log')
end

subplot(2,1,1); legend(names(IDX(1,:)),'Location','NorthEast');
% extents = [xBnd,0,45];  axis(extents);
subplot(2,1,2); legend(names(IDX(2,:)),'Location','NorthWest');
% extents = [xBnd,30,60];

%%%% SAVE %%%%
save('DATA_Slip_vs_Mu.mat','Data');
save2pdf('../WriteUp/Figures/Slip_vs_Mu__Distance.pdf',H_dist,600);
save2pdf('../WriteUp/Figures/Slip_vs_Mu__Angle.pdf',H_angle,600);