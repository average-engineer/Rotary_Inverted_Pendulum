clearvars
close all
clc
format short

%% Rotary Arm
M = 0.095;
L = 0.085;

%% Inverted Pendulum
m = 0.024;
l = 0.129;

%% Gravity
g = 9.81;

%% Simulation time vector
dt = 0.01; % Time Step Size
t_span = [0:dt:5];

%% Degrees of Freedom of the System
dof = 2;

%% State Vector of the system
% [Rotary Arm Displacement ; Pendulum Displacement ; Rotary Arm Velocity ; Pendulum Velocity ]
%% Initial State Vector
w_0 = [0;-0.05;0;0];

%% SWITCHING PARAMETER OF BALANCE CONTROL
% If the initial pendulum angle < +-10 degs, then the balance control is
% activated i.e. PD control is activated and the motor applied the required
% torque to balance the inverted pendulum and bring the rotary arm to its
% zero position
% If initial pendulum angle > +-10, then the PD controller is not activated and the
% system remains unforced/free (and naturally unstable)

if w_0(2) > 10*pi/180 || w_0(2) < -10*pi/180
    sys = 'OpenLoop';
    fprintf('The system is unforced');
else
    sys = 'ClosedLoop';
    fprintf('The system will be actuated by the Motor');
end

%% Desired State Vector
% [Desired Arm Posn;Desired Pendulum Position;Desired Arm Velocity;Desired Pendulum Velocity]
for ii = 1:length(t_span)
    wd(:,ii) = zeros(2*dof,1);
%     wd(1,ii) = 0.01*sin(t_span(ii));
%     wd(2,ii) = 0.01*cos(t_span(ii));
%     wd(3,ii) = 0.01*cos(t_span(ii));
%     wd(4,ii) = -0.01*sin(t_span(ii));
end

%% PD Controller Parameters
% Proportional Gain
Kp = [-2.2361,53.8740];

% Derivative Gain
Kd = [-1.7400,6.5269];


% V = [-1 + 1i;-1 - 1i;-100 - 100*1i;-100 + 100*1i];
% 
% K = place(A,B*[0;1],V);
% 
% Kp = [K(1) K(2)];
% Kd = [K(3) K(4)];

%% Computing Dynamics
% variable for deciding actuation :sys

switch sys
    case 'OpenLoop'
        %% Open Loop Dynamics
        % Unforced system: no motor torque or any disturbance
        [t,w] = ode45(@(t,w)OpenLoopDyn(t,w,m,M,L,l,g),t_span,w_0);
        
    case 'ClosedLoop'
        %% Linearlized system dyanamics
        % (System linearlized about the unstable point of inverted pendulum)
        % For regulator design
        [t,w] = ode45(@(t,w)ClosedLoopDyn1(t,w,m,M,L,l,g,dof,Kp,Kd,wd),t_span,w_0);
        
        % Mass Matrix
        M_mat = [((M/3) + m)*(L^2) ,-(m*L*l)/(2);
            -(m*L*l)/(2),(m*(L^2))/(3)];
        
        % Stiffness Matrix
        K_mat = [0,0;
            0,-(m*g*l)/2];
        
        % Open Loop State Weighing matrix (for linear system)
        A = [zeros(dof,dof),eye(dof,dof);
            -inv(M_mat)*K_mat,zeros(dof,dof)];
        
        % Control Cost Matrix
        B = [zeros(dof,dof);M_mat\eye(size(M_mat))];
        
        %% For Regulation Problem
        % Closed Loop State Weighing Matrix
        Acl = A + [-B*[1;0]*Kp,-B*[1;0]*Kd]; % Only for regulation
        % Acl1 = A - B*[0;1]*K;
        % eig(Acl1)
        
        % Closed Loop Poles
        closedlooppoles = eig(Acl)
        
        %% Controllability of the system
        P = ctrb(A,B*[1;0]);
        CtrbTestMat_rank = rank(P)
        
        %% Actuator Effort
        % There is only one actuator present at the rotary arm end
        for ii = 1:length(t_span)
            u(ii) = Kp*(wd(1:2,ii) - [w(ii,1);w(ii,2)]) - Kd*(wd(3:4,ii) - [w(ii,3);w(ii,4)]);
        end
        
        figure
        plot(t_span,u,'linewidth',2)
        grid on
        xlabel('Time(s)')
        ylabel('Motor Torque (Nm)')
end


%% Responses
figure
subplot(1,2,1)
plot(t_span,w(:,1),'linewidth',2)
grid on
xlabel('Time(s)')
ylabel('Rotary Arm Angle (rad)')

subplot(1,2,2)
plot(t_span,w(:,2),'linewidth',2)
grid on
xlabel('Time(s)')
ylabel('Pendullum Angle (rad)')


%% Inverse Dynamics Verification
Q = Inv_Dynamics_LE_RotInvPend(w,dof,[L;l],[M;m],t_span,g);

% Motor Torque
Qmotor = Q(:,1);

% Comparing the torque with the fwd dynamics motor control torque (control
% law)
figure
hold on
plot(t_span,u,'linewidth',2)
plot(t_span,Qmotor,'linewidth',2)
legend('Control Law','Inv Dynamics Torque')
grid on
xlabel('Time(s)')
ylabel('Motor Torque (Nm)')