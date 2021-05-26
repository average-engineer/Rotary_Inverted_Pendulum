clearvars
close all
clc

%% Rotary Arm
M = 1;
L = 0.5;

%% Inverted Pendulum
m = 0.5;
l = 1;

%% Gravity
g = 9.81;

%% Simulation time vector
t_span = [0:0.01:5];

%% Degrees of Freedom of the System
dof = 2;

%% Dynamic Parameters
% Mass Matrix
M_mat = [((M/3) + m)*(L^2),-(m*L*l)/(2);
        -(m*L*l)/(2),(m*(l^2))/(3)];
    
% Damping Matrix
C_mat = [0,0;
        0*(m*L*l)/2,0];
    
% Stiffness Matrix
K_mat = [0,0;
        0,-(m*g*l)/2];
    
% State Weighing Matrix
A = [zeros(dof,dof),eye(dof,dof);
     -M_mat\eye(size(M_mat))*K_mat,-M_mat\eye(size(M_mat))*C_mat];
 
B = [zeros(dof,dof);M\eye(size(M_mat))];

%% State Vector of the system
% [Rotary Arm Displacement ; Pendulum Displacement ; Rotary Arm Velocity ; Pendulum Velocity ]
%% Initial State Vector
w_0 = [0.5;0.05;0;0];


%% Controllability of the system
P = ctrb(A,B*[0;1]);
rank(P)

%% PD Controller Parameters
% Proportional Gain
Kp = [2,1];

% Derivative Gain
Kd = [2,1];


% V = [-1 + 1i;-1 - 1i;-100 - 100*1i;-100 + 100*1i];
% 
% K = place(A,B*[0;1],V);
% 
% Kp = [K(1) K(2)];
% Kd = [K(3) K(4)];

%% Computing Dynamics
[t,w] = ode45(@(t,w)ClosedLoopDyn1(t,w,m,M,L,l,g,dof,Kp,Kd),t_span,w_0);

% Mass Matrix
M_mat = [((M/3) + m)*(L^2),-(m*L*l)/(2);
     -(m*L*l)/(2),(m*(L^2))/(3)];
 
% Stiffness Matrix
K_mat = [0,0;
     0,-(m*g*l)/2];
%% Open Loop State Weighing Matrix
A = [zeros(dof,dof),eye(dof,dof);
     -M_mat\eye(size(M_mat))*K_mat,zeros(dof,dof)];
% Control Cost Matrix
B = [zeros(dof,dof);M\eye(size(M_mat))];
 
% Closed Loop State Weighing Matrix
Acl = A + [-B*[0;1]*Kp,-B*[0;1]*Kd];
% Acl1 = A - B*[0;1]*K;
% eig(Acl1)

% Closed Loop Poles
eig(Acl)

%% Responses
% figure
% plot(t_span,w())