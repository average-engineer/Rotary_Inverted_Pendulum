function dwdt = ClosedLoopDyn1(t,w,m,M,L,l,g,n,Kp,Kd)

% State Space Model of the Regulator problem of the linearlized system of
% inverted rotary pemdulum

% % Mass Matrix
% M_mat = [((M/3) + m)*(L^2),-(m*L*l)*cos(w(1))/(2);
%         -(m*L*l*cos(w(1)))/(2),(m*(l^2))/(3)];
%     
% % Damping Matrix
% C_mat = [0,0;
%         (m*L*l*sin(w(1))*w(3))/2,0];
%     
% % Stiffness Matrix
% K_mat = [0,0;
%         0,-(m*g*l)/2];
%     
% % State Weighing Matrix
% A = [zeros(n,n),eye(n,n);
%      -M_mat\eye(size(M_mat))*K_mat,-M_mat\eye(size(M_mat))*C_mat];
%  
% B = [zeros(n,n);M\eye(size(M_mat))];

% Dynamic Parameters
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
A = [zeros(n,n),eye(n,n);
     -M_mat\eye(size(M_mat))*K_mat,-M_mat\eye(size(M_mat))*C_mat];
 
B = [zeros(n,n);M\eye(size(M_mat))];

% Desired Positions
qd = [0.5;0];
qd_dot = [0;0];

% Generalized Coordinates
q = [w(1);w(2)];

% Generalized Velocities
q_dot = [w(3);w(4)];

% Closed Loop System Input
% Only the Rotary Arm is actuated
% u = [Kp*(qd - q) + Kd*(qd_dot - q_dot);0];
u = [0;Kp*(qd - q) + Kd*(qd_dot - q_dot)];

% State Space Model
dwdt = A*w + B*u;
end