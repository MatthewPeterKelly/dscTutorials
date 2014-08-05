function cost = Objective_Function(x,P)

%Special case - no cost function
if P.MS.cost_function==0
    cost = 0;
    return
end

%Expand the decision variable vector to a struct
X = convertVecStruct(x,P.structDat);

switch P.MS.cost_function
    case 1 %force squared

        uLow = X.force(1:(end-1));
        uUpp = X.force(2:end);
        uMid = 0.5*(uLow + uUpp);
        
        cLow = uLow.^2;
        cMid = uMid.^2;
        cUpp = uUpp.^2;
        
        %Time step
        h = X.duration/(P.MS.nGrid-1);
        
        %Use simpson's rule to approximate cost:
        cost = (h/6)*sum(cLow + 4*cMid + cUpp);
        
    otherwise
        error('Invalid cost function type')
        
end

end
