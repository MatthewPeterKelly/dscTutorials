function B = B_Matrix(phi,th,psi,v,Lt,Lc)

%This function is the jacobian of the system dynamics with respect to the
%actuators. It is derived in Derive_EoM.m

B = [...
 
                   -cos(phi)*cos(psi)*sin(th),                      v*cos(phi)*sin(psi)*sin(th);
                    cos(phi)*cos(psi)*cos(th),                     -v*cos(phi)*cos(th)*sin(psi);
                       (cos(psi)*sin(phi))/Lt,                        -(v*sin(phi)*sin(psi))/Lt;
 (Lt*sin(psi) - Lc*cos(psi)*sin(phi))/(Lc*Lt), (v*(Lt*cos(psi) + Lc*sin(phi)*sin(psi)))/(Lc*Lt)];

end