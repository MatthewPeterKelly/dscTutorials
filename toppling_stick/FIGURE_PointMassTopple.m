%FIGURE_PointMassTopple

%This is still experimental - does not produce a meaningful plot yet

P.m = 1; P.g = 1; P.L = 1; 

small = 1e-6;

n = 50;
MoI = logspace(-3,0,n);
Th = linspace(0+small,pi/2-small,n);
[MOMENT,THETA] = meshgrid(MoI,Th);
FRICTION = zeros(size(MOMENT));    %Critical value of mu

for i=1:size(MOMENT,1)
    for j=1:size(MOMENT,2)
        P.I = MOMENT(i,j);
        th = THETA(i,j);
        FRICTION(i,j) = topple_criticalMu(th,P);
    end
end

[N,M] = size(MOMENT);
moi = reshape(MOMENT,N*M,1);
mu = reshape(FRICTION,N*M,1);
th = reshape(THETA,N*M,1);

figure(700); clf; hold on;
idx = mu>0;
plot3(moi(idx),th(idx),mu(idx),'r.');
idx = mu<0;
plot3(moi(idx),th(idx),-mu(idx),'b.');
set(gca,'Xscale','log')
set(gca,'Yscale','linear')
set(gca,'Zscale','log')
xlabel('moi')
ylabel('angle')
zlabel('mu')