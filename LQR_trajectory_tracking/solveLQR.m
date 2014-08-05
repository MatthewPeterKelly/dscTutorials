function T = solveLQR(T)

N = length(T);   %Number of points along trajectory

Nx = size(T(1).x,1);  %Dimension of the state 
Nu = size(T(1).u,1);  %Dimension of the input

T(N).P = zeros(Nx,Nx); %Initialize cost to go
T(N).K = zeros(Nu,Nx);   %Last gain matrix is trivial

for i=1:(N-1);
    idx = N-i;
    P = T(idx+1).P;
    Q = T(idx).Q;
    R = T(idx).R;
    A = T(idx).A;
    B = T(idx).B;
    
    K = -(R+B'*P*B)\(B'*P*A);
    Pnext = Q + K'*R*K + (A+B*K)'*P*(A+B*K);
    
    T(idx).P = Pnext;
    T(idx).K = K;   
end
