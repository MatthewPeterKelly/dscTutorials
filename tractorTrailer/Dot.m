function Val = Dot(A,B)

%2 dimensionsl dot product

% A and B must both be of size (2xN);

Val = A(1,:)*B(1,:) + A(2,:)*B(2,:);


%NOTE - old version was not vectorized. Revert to that if errors.

end