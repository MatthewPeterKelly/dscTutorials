function EoM_slide()
%
%Derives the equations of motion for a rigid body sliding with coulomb
%friction on a horizontal surface
%
%

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                             Parameters                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

syms m I L g u d 'real'
% m = mass of the rigid object
% I = moment of inertia about the center of mass
% L = distance between the tip of the object (O) and center of mass (G)
% g = acceleration due to gravity
% u = coefficient of friction
% d = direction of slide. true => positive, false => negative

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        States and Unknowns                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

syms th x dth dx 'real'
% th = angle between [position of G wrt O] and positive vertical axis
% x = horizontal position of point O
% dth = time derivative of th
% dx = time derivative of x

syms ddth ddx H V 'real'
% ddth = second time derivative of th
% ddx = second time derivative of x
% H = horizontal component of the contact force
% V = vertical component of the contact force

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Coordinate System                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Inertial reference frame
i = sym([1;0;0]);
j = sym([0;1;0]);
k = sym([0;0;1]);

%Rotating reference frame
er = cos(th)*j + sin(th)*(-i);   %Unit vector along the line G wrt O
et = -sin(th)*j + cos(th)*(-i);  %Unit vector normal to er in the plane

%Frame derivatives
der = dth*et;
det = -dth*er;
dder = ddth*et + dth*det;

%Position Vectors for CoM
rg = x*i + L*er;
drg = dx*i + L*der;
ddrg = ddx*i + L*dder;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%           Conservation of angular momentum rate about origin            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% sum of moments = rate of change in angular momentum
Mo = cross(rg, -m*g*j) + cross(x*i,V*j);   %Sum of moments about origin
dHo = I*ddth + cross(rg, m*ddrg);   %Rate of change of angular momentum
Eqn1 = simplify(dot(Mo-dHo,k));    %Extract k component and simplify

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Conservation of linear momentum rate                    %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% sum of forces = rate of change in momentum
sum_forces = H*i + V*j - m*g*j;
momentum_rate = m*ddrg;
eqn = simplify(sum_forces-momentum_rate);
Eqn2 = dot(eqn,i);
Eqn3 = dot(eqn,j);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Coefficient of Friction                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

Eqn4_pos = H + u*V; % If dx > 0
Eqn4_neg = H - u*V; % If dx < 0

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Mechanical Energy of the system                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

Energy.potential = -dot(rg,-m*g*j);
Energy.kinetic = (1/2)*m*dot(drg,drg) + (1/2)*I*dth^2;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Solve equations of motion                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

Soln_pos = solve(Eqn1,Eqn2,Eqn3,Eqn4_pos,ddth,ddx,H,V);
Soln_pos.ddth = simplify(Soln_pos.ddth);
Soln_pos.ddx = simplify(Soln_pos.ddx);
Soln_pos.H = simplify(Soln_pos.H);
Soln_pos.V = simplify(Soln_pos.V);

Soln_pos.inf.ddth = limit(Soln_pos.ddth,u,inf);
Soln_pos.inf.ddx = limit(Soln_pos.ddx,u,inf);
Soln_pos.inf.H = limit(Soln_pos.H,u,inf);
Soln_pos.inf.V = limit(Soln_pos.V,u,inf);

Soln_pos.zero.ddth = limit(Soln_pos.ddth,u,0);
Soln_pos.zero.ddx = limit(Soln_pos.ddx,u,0);
Soln_pos.zero.H = limit(Soln_pos.H,u,0);
Soln_pos.zero.V = limit(Soln_pos.V,u,0);

Soln_neg = solve(Eqn1,Eqn2,Eqn3,Eqn4_neg,ddth,ddx,H,V);
Soln_neg.ddth = simplify(Soln_neg.ddth);
Soln_neg.H = simplify(Soln_neg.H);
Soln_neg.V = simplify(Soln_neg.V);

Soln_neg.inf.ddth = limit(Soln_neg.ddth,u,inf);
Soln_neg.inf.ddx = limit(Soln_neg.ddx,u,inf);
Soln_neg.inf.H = limit(Soln_neg.H,u,inf);
Soln_neg.inf.V = limit(Soln_neg.V,u,inf);

Soln_neg.zero.ddth = limit(Soln_neg.ddth,u,0);
Soln_neg.zero.ddx = limit(Soln_neg.ddx,u,0);
Soln_neg.zero.H = limit(Soln_neg.H,u,0);
Soln_neg.zero.V = limit(Soln_neg.V,u,0);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Write Function File                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
writeSlide(Soln_pos, Energy, 'dynamics_slidePos')
writeSlide(Soln_neg, Energy, 'dynamics_slideNeg')

end