% MAIN - Dynamics Function Gradients
%
% This script demonstrates how to use the functions for computing gradients
% of dynamics functions, as well as testing that the results match the
% analytic solutions.
%
%
% See derivations in Derive_Equations
%
% Core of the gradient of the backslash is from:
%     http://www.atmos.washington.edu/~dennis/MatrixCalculus.pdf

clc; clear;

N = 100;
q = randn(2,N);
dq = randn(2,N);
u = rand(3,N);

tic
[A_ddq, A_ddqz]  = dynamicsAnalytic(q,dq,u);
toc
tic
[N_ddq, N_ddqz]  = dynamicsNumeric(q,dq,u);
toc

error_dyn = A_ddq-N_ddq;
error_grad = A_ddqz-N_ddqz;

fprintf('Max error in dynamics: %6.6g\n',max(max(abs(error_dyn))));
fprintf('Max error in gradients: %6.6g\n',max(max(max(abs(error_grad)))));
