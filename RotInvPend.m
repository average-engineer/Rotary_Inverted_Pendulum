clearvars
close all
clc
format short

%% Rotary Arm
% M = 0.095;
% L = 0.085;
M = 2;
L = 1;

%% Inverted Pendulum
% m = 0.024;
% l = 0.129;
m = 1;
l = 2;

%% Gravity
g = 9.81;

%% Simulation time vector
dt = 0.01; % Time Step Size
t_span = [0:dt:1000000];

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
wd = [0;0;0;0];

%% PD Controller Parameters
% Proportional Gain
Kp = [-2.2361,53.8740]; % Fast Response
% Kp = [-2,1];
% Kp = [-0.5,50]; % Slow Response
% Derivative Gain
% Kd = [-20,2.5];
Kd = [-1.7400,6.5269]; % Fast Response
% Kd = [2,-1]; % Slow Response

%% External Disturbance Force
dist = 'None'; % Variable for setting the disturbance

switch dist
    case 'None'
        fdist = zeros(2,length(t_span));
    case 'Impulse'
        fdist = ImpulseForce(t_span,5,100,dt,dof);
    case 'Harmonic'
        fdist = zeros(2,length(t_span));
        fd1 = 2*sin(t_span);
        fdist(1,:) = fd1;
    case 'Static'
        fdist = zeros(2,length(t_span));
        fdist(1,:) = 2;
    case 'Ramp'
        fdist = zeros(2,length(t_span));
        fdist(1,:) = 2*t_span;
end


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
        [t,w] = ode45(@(t,w)ClosedLoopDyn1(t,w,m,M,L,l,g,dof,Kp,Kd,wd,dist),t_span,w_0);
        
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
        CtrbTestMat_rank = rank(P);
        if CtrbTestMat_rank == 2*dof
            fprintf('The system is controllable');
        else
            fprintf('The system is not controllable');
        end
        
        %% Actuator Effort
        % There is only one actuator present at the rotary arm end
        for ii = 1:length(t_span)
            u(:,ii) = [Kp*(wd(1:2) - [w(ii,1);w(ii,2)]) + Kd*(wd(3:4) - [w(ii,3);w(ii,4)]) ; 0] + fdist(:,ii);
            u1(ii) = u(1,ii);
            motorTorque(ii) = u1(ii) - fdist(1,ii);
            u2(ii) = u(2,ii);
        end
        
        figure
        subplot(3,1,1)
        plot(t_span,u1,'linewidth',2)
        grid on
        xlabel('Time(s)')
        ylabel('Torque (Nm)')
        title('Rotary Arm - Base Actuator Torque (Nm)')
        
        subplot(3,1,2)
        plot(t_span,u2,'linewidth',2)
        grid on
        xlabel('Time(s)')
        ylabel('Torque (Nm)')
        title('Rotary Arm - Inverted Pendulum Joint Torque (Nm)')
        
        subplot(3,1,3)
        plot(t_span,motorTorque,'linewidth',2)
        grid on
        xlabel('Time(s)')
        ylabel('Torque (Nm)')
        title('Motor Torque (Nm)')
end


%% Responses
figure
subplot(2,1,1)
% plot(t_span,rad2deg(w(:,1)),'linewidth',2)
plot(t_span,w(:,1),'linewidth',2)
grid on
xlabel('Time(s)')
ylabel('Rotary Arm Angle (rad)')

subplot(2,1,2)
% plot(t_span,rad2deg(w(:,2)),'linewidth',2)
plot(t_span,w(:,2),'linewidth',2)
grid on
xlabel('Time(s)')
ylabel('Pendullum Angle (rad)')
