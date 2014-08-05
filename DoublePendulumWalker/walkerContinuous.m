function output = walkerContinuous(input)

auxdata = input.auxdata;

%-------------------------------------------------------------------------%
%--------------------- Equations of Motion -------------------------------%
%-------------------------------------------------------------------------%

t = input.phase(1).time;
X = input.phase(1).state;
control = input.phase(1).control;
disturbance = zeros(length(t),auxdata.index.NUMBER_OF_DISTURBANCES);
U = [control,disturbance];

P = auxdata.dynamics;   %Parameters for the continuous dynamics

output(1).dynamics = Double_Pendulum_Dynamics(t',X',U',P)'; 


%-------------------------------------------------------------------------%
%------------------------ Cost Integral ----------------------------------%
%-------------------------------------------------------------------------%
idx = auxdata.index;

switch auxdata.cost.model
    case 'TorqueSquared' %Time Integral of Torque^2 
        output(1).integrand = control.^2;

    case 'CostOfTransport' %Time Integral of Torque*Rate
       
        hipRate = X(:,idx.DPHI)-X(:,idx.DTHETA);
        ankRate = X(:,idx.DTHETA);
        
        hipTorque = control(:,idx.HIP);
        ankTorque = control(:,idx.ANK);
        
        %Work = Integral(time derivative of work)
        dHipWork = hipRate.*hipTorque;
        dAnkWork = ankRate.*ankTorque;
        
        %Collect the integrals
        integrand = zeros(size(control));
        integrand(:,idx.HIP) = dHipWork;
        integrand(:,idx.ANK) = dAnkWork;
        
        %Smoothing and weighting
        alpha = auxdata.cost.workSmoothing;
        beta = auxdata.cost.negativeWork;
        output(1).integrand = SmoothAbs(integrand,alpha,beta,1); 

    otherwise
        error('Invalid Cost Method');
end


end