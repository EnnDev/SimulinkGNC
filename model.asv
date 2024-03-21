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

%our TF - closed loop
TF = tf(syscl);
TFc = TF(2,1);

%LQG Kalman filter design
G = eye(2);
H = 0*eye(2)

%Kalman Q,R noise matrices
Qbar = diag(0.00015*ones(1,2));
Rbar = diag(0.55*ones(1,2));

%define noisy system
sys_n = ss(A,[B G],C,[D H]);
[kest.L.P] = kalman(sys_n,Qbar,Rbar,0);

%kalmain gain observer closed loop
Aob = A-L*C;

%display observer eigenvalues
fprintf('observer eigenvalues\n');
disp(eig(Aob));

%%noise time constants (you chooese)
dT1 = 0.75;
dT2 = 0.25;

% missile/model parameters

R = 6371e3; %Earth Radius
Vel = 1021.08; %speed (m/s)
m2f = 3.2811; %meters to feet

%target location
LAT_TARGET = 34.6588;
LONG_TARGET = -119.4573;
ELEV_TARGET = 10000; %m p - MSL

%initial location
LAT_INIT
LONG_INIT
ELEV_INIT

%obstacle location
LAT_OBS
LONG_OBS
ELEV_OBS

