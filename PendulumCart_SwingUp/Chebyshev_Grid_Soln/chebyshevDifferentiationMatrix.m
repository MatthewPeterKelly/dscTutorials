function D = chebyshevDifferentiationMatrix(n,d)

if nargin==1
    d = [-1,1];  %Default domain
end

%Get the chebyshev points
    x = chebyshevPoints(n,d);

    %Get the weight vector
    w = ones(1,n);
    w(2:2:n) = -1;
    w([1,end]) = w([1,end])/2;

    %First, compute the weighting matrix:
    W = (1./w)'*w;

    %Next, compute the matrix with inverse of the node distance
    X = zeros(n);
    for i=1:n
        idx = (i+1):n;
        X(i,idx) = 1./(x(i)-x(idx));
    end

    %Use the property that this matrix is anti-symetric
    X = X - X';

    %Compute the i~=j case:
    D = W.*X;

    %Deal with the i=j case:
    D = D - diag(sum(D,2));
    
end