function EoM_nLink_pendulum(N,overwrite)
%EoM_nLInk_Pendulum.m
%
% N = number of links in pendulum

% This script derives the equations of motion for a pendulum with n links.

% Model: Each link has a mass, length, moment of inertia, and center of
% mass,

% The lagrangian (L) is defined as:
%
%       L = T - U
%
% where
%       T = system's kinetic energy
%       U = system's potential energy

% How we go about expressing the equations of motion:
%
%       DL      D  / DL \         * Note that some of those 'D' should be
%       ---  =  -- | -- |           curvy 'D' to represent partial
%       Dq      Dt \ Dq /           derivatives

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        set up variables                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

filename = ['dynamics_' num2str(N) '_link.m'];

if N<2 || N>10
    error('N must be an integer in 2:10')
elseif ~overwrite && exist(filename,'file');
     fprintf(['WARNING: File: ' filename ' already exists. Did not re-derive EoM.\n']);  
     success = false;
else  %Then go ahead and do lots of math
    
    %Create state variables:
    th = sym('th',[N,1]); sym(th,'real');   %Absolute angle of each link
    dth = sym('dth',[N,1]); sym(dth,'real');  %Angular rate of each link
    ddth = sym('ddth',[N,1]); sym(ddth,'real');  %Angular accelerations
    
    %Create parameters:
    g = sym('g','real');  %Gravity
    m = sym('m',[N,1]); sym(m,'real'); %mass of each link
    l = sym('l',[N,1]); sym(l,'real'); %length of each link
    d = sym('d',[N,1]); sym(d,'real'); %distance between CoM and parent joint
    I = sym('I',[N,1]); sym(I,'real'); %moment of inertia of each link about its CoM
    
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                            vector stuff                                 %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    
    %Unit vectors for each link:
    e = [cos(th), sin(th)]; %Unit vector along each link
    n = [-sin(th), cos(th)]; %Unit vector normal to each link
    
    %Derivatives of unit vectors:
    de = [dth dth].*n; %d/dt Unit vector along each link
    dn = -[dth dth].*e; %d/dt Unit vector normal to each link
    
    %Position vectors:
    % % G = sym('G',[N,2]); sym(G,'real'); %Center of mass positions
    % % dG = sym('dG',[N,2]); sym(dG,'real'); %Center of mass velocities
    % % P = sym('P',[N,2]); sym(P,'real'); %End of link positions
    % % dP = sym('dP',[N,2]); sym(dP,'real'); %End of link velocities
    
    %Compute relative positions and velocities:
    G= [d d].*e;
    dG = [d d].*de;
    P = [l l].*e;
    dP = [l l].*de;
    
    %Convert to absolute positions and velocities
    for i = 2:N
        G(i,:) = G(i,:) + P(i-1,:);
        dG(i,:) = dG(i,:) + dP(i-1,:);
        P(i,:) = P(i,:) + P(i-1,:);
        dP(i,:) = dP(i,:) + dP(i-1,:);
    end
    
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                          Lagrangian Definitions                         %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    
    %Kinetic energy:
    T = sum((1/2)*m.*(dG(:,1).*dG(:,1) + dG(:,2).*dG(:,2))) + ...
        sum((1/2)*I.*(dth.*dth));
    
    %Potential energy:
    yIndex = 2;
    U = sum(m.*g.*G(:,yIndex));
    
    %Lagrangian:
    L = T - U;
    
    %Generalized coordinates:
    q = th;
    dq = dth;
    ddq = ddth;
    
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                  evaluate partial derivatives                           %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    
    %              DL
    %  DL_Dq  ==  ---      Note that 'D' is partial derivative here
    %              Dq
    %
    DL_Dq = jacobian(L,q')';
    
    %              DL
    %  DL_Ddq  ==  ---      Note that 'D' is partial derivative here
    %              Ddq
    %
    DL_Ddq = jacobian(L,dq')';
    
    %                D  / DL  \         * Note that some of those 'd' should be
    % DDL_DtDdq  ==  -- | --  |         curvy 'D' to represent partial
    %                Dt \ Ddq /         derivatives
    %
    % Note the application of the chain rule:  (Quoting Andy Ruina: )
    %      d BLAH / dt  =  jacobian(BLAH, [q qdot])*[qdot qddot]'
    %
    DX_DQ = jacobian(DL_Ddq,[q; dq]);
    DQ_DT = [dq; ddq];
    
    DDL_DtDdq = DX_DQ * DQ_DT;
    
    
    %Write out as single equation and simplify:
    EoM = DL_Dq - DDL_DtDdq;
    EoM = simplify(EoM);
    
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                   mass matrix gymnastics                                %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    
    % This is a confusing step - read carefully:
    %
    % We know that our equations are of the form:
    %             EoM = M(q,dq)*qdd + f(q,dq) == 0;
    %
    % Thus, we can find M(q,dq) by using the jacobian command:
    
    M = simplify(jacobian(EoM,ddq));
    
    % Now, we want to find f(q,dq). We can do this by substituting in zero for
    % the acceleration vector (dqq)
    
    f = simplify(subs(EoM,ddq,zeros(size(ddq))));
    
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                               write files                               %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    
    writeDynamics(f,M);
    writeEnergy(N,T,U);
    writePosition(N,P,G);  
end

end
