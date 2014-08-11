function A = A_Matrix(phi,th,psi,v,Lt)

%This is the jacobian of the system dynamics. These equations are derived
%in the function Derive_EoM.m

A = [...
 
 0, 0, -v*cos(phi)*cos(psi)*cos(th),  v*cos(psi)*sin(phi)*sin(th);
 0, 0, -v*cos(phi)*cos(psi)*sin(th), -v*cos(psi)*cos(th)*sin(phi);
 0, 0,                            0,     (v*cos(phi)*cos(psi))/Lt;
 0, 0,                            0,    -(v*cos(phi)*cos(psi))/Lt];

end