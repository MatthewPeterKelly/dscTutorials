function cost = Objective_Function(x,P)

%Special case - no cost function
if P.MS.cost_function==0
    cost = 0;
    return
end

%Expand the decision variable vector to a struct
X = convertVecStruct(x,P.structDat);

%Domain of the problem:
domain = [0,X.duration];

switch P.MS.cost_function
    case 1 %force squared

        %get the integrand value at each grid point:
        costIntegrand = X.force.^2;

        
    case 2  %total work (positive and negative)

        %get the integrand value at each grid point:
        States = [P.MS.Start, X.state, P.MS.Finish];
        work = X.force.*States(2,:);   %force * velocity
        alpha = P.MS.smoothing;
        order = 2;  %Second order smooth should be fine for our purposes
        costIntegrand = smoothAbs(work,alpha,order);

    case 3  % integral of absolute force
        alpha = P.MS.smoothing;
        order = 2;  %Second order smooth should be fine for our purposes
        costIntegrand = smoothAbs(X.force,alpha,order);
        
    case 4  % integral of (derivative of force)^2
        costIntegrand = chebyshevDerivative(X.force, domain).^2;
        
    case 5 %force ^ 4

        %get the integrand value at each grid point:
        costIntegrand = X.force.^4;
        
        
    otherwise
        error('Invalid cost function type')
        
end

%Use Clenshaw-Curtis quadrature on the chebyshev grid to compute integral
If = chebyshevIntegral(costIntegrand,domain);
cost = If(end);

end
