clc;
clear all;
close all;

%% Motor Parameters

Rm = 8.4;         % Motor Resistance
kt = 0.042;       % Current-torque (N-m/A)
km = 0.042;       % Back-emf constant (V-s/rad)

%% Rotary Arm Parameters

Mr = 0.095;       % Mass (kg)
Lr = 0.085;       % Total length (m)
Jr = Mr*Lr^2/12;  % Moment of inertia about pivot (kg-m^2)
Dr = 0;           % Equivalent Viscous Damping Coefficient (N-m-s/rad)

%% Pendulum Link Parameters

Mp = 0.024;       % Mass (kg)
Lp = 0.129;       % Total length (m)
Jp = Mp*Lp^2/12;  % Moment of inertia about pivot (kg-m^2)
Dp = 0;           % Equivalent Viscous Damping Coefficient (N-m-s/rad)
g  = 9.81;        % Gravity Constant


%% Initial Value

Theta_initial=20*pi/180;  % Initial Value of Theta at time = 0 sec
Alpha_initial=0*pi/180;   % Initial Value of Alpha at time = 0 sec
Theta_desired=0*pi/180;   % Desired value of Theta at Stabilization

%% Controller Parameters

Kp_Theta = -2.2361;    % Propostional Gain for Theta
Kd_Theta = -1.7400;    % Derivative Gain for Theta
Ki_Theta = -0;         % Integral Gain for Theta

% Putting Ki_Theta = -1 will remove steady state error due to disturbance.

Kp_Alpha = 53.8740;    % Propostional Gain for Alpha
Kd_Alpha = 6.5269;     % Derivative Gain for Alpha
Ki_Alpha = 0;          % Integral Gain for Alpha

%% Simulation Results

sim('PIDControl1.slx');
Sth = stepinfo(eth*180/pi,time,Theta_desired)
Sal = stepinfo(eal*180/pi,time)

displayperformance=1

if displayperformance==0

figure('units','normalized','outerposition',[0 0 1 1])

subplot(2,2,1);plot(time,eth*180/pi,'LineWidth',2); grid on;
xlabel('time (sec)');
ylabel('Error in Theta (deg)');


subplot(2,2,2);plot(time,eal*180/pi,'LineWidth',2); grid on;
xlabel('time (sec)');
ylabel('Error in Alpha (deg)');


else


figure('units','normalized','outerposition',[0 0 1 1])

subplot(2,2,1);plot(time,eth*180/pi,'LineWidth',2); grid on;
xlabel('time (sec)');
ylabel('Error in Theta (deg)');


dim = [0.2 0.5 0.5 0.3];
str = {['Rise Time = ',num2str(Sth.RiseTime)],['Settling Time = ',num2str(Sth.SettlingTime)], ['Overshoot = ', num2str(Sth.Peak)]};
annotation('textbox',dim,'String',str,'FitBoxToText','on');

subplot(2,2,2);plot(time,eal*180/pi,'LineWidth',2); grid on;
xlabel('time (sec)');
ylabel('Error in Alpha (deg)');

dim = [0.7 0.5 0.5 0.3];
str = {['Rise Time = ',num2str(Sal.RiseTime)],['Settling Time = ',num2str(Sal.SettlingTime)], ['Overshoot = ', num2str(Sal.Peak)]};
annotation('textbox',dim,'String',str,'FitBoxToText','on');

end





