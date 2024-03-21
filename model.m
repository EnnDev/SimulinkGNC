clc; close all; clear all;
format short

% 
A = [-1.064 1.000; 290.6 0.00];
B = [-0.25; -331.40];
C = [-123.34 0.00; 0.00 1.00];
D = [-13.51; 0.00];

states = {'AoA', 'q'};
inputs = {'\delta_c'};
outputs = {'Az', 'q'};

sys = ss(A,B,C,D,'statename',states,...
    'inputname',inputs,...
    'outputname',outputs);

%TF
TFs = tf(sys);
TF = TFs(2,1);
disp(pole(TF));

%LQR weight matrices
Q = [0.1 0; 0 0.1]
R = 0.5;

%LQR gain
[K,S,e] = lqr(A,B,Q,R);
fprintf('eigenvalues of A-BK\n');
dis(eig(A-B*K));
fprintf(['Feedback gain K)');
disp(K)

%closed loop system
Acl = A-B*K;
Bcl = B;

syscl = ss(Acl,Bcl,C,D,'statename',states,...
    'inputname',inputs,...
    'outputname',outputs);