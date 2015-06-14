function drawCartPole(t,z,dyn)
%drawCartPole(t,z,dyn)
%
% This function draws the cart pole's current state


cartWidth = 0.3*dyn.l;
cartHeight = 0.2*dyn.l;

extents = [-3.5,1,-cartHeight - 1.2*dyn.l, cartHeight + 1.2*dyn.l];

cartColor = [33,87,177]/256;
poleColor = [183,27,73]/256;
groundColor = [84,64,64]/256;

x = z(1,:);
q = z(2,:);
dx = z(3,:);
dq = z(4,:);

p = autoGen_cartPoleKinematics(x,q,dx,dq,dyn.l);

p1 = [x;0];   %Center of cart
p2 = p;   % Tip of pendulum

clf; hold on;
axis equal; axis(extents);

% Draw the cart:
h = rectangle(...
    'Position',[(p1'-0.5*[cartWidth,cartHeight]),[cartWidth,cartHeight]],...
    'Curvature',0.2*[1,1],...
    'EdgeColor',cartColor,...
    'LineWidth',1,...
    'FaceColor',cartColor);

plot(extents(1:2),-0.55*cartHeight*[1,1],'LineWidth',4,'Color', groundColor);

pos = [p1,p2];
plot(pos(1,:),pos(2,:),'Color',poleColor,'LineWidth',3);
plot(p2(1),p2(2),'.','Color',poleColor','MarkerSize',80);

title(['Cart-Pole,  t = ', num2str(t,2)]);

end