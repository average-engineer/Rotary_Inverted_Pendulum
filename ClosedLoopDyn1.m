function dwdt = ClosedLoopDyn1(t,w,m,M,L,l,g,n,Kp,Kd,wd,disturbance)

% State Space Model of the Regulator problem of the linearlized system of
% inverted rotary pemdulum
% System has been linearlized

% Dynamic Parameters
% Mass Matrix
M_mat = [((M/3) + m)*(L^2) ,-(m*L*l)/(2);
        -(m*L*l)/(2),(m*(l^2))/(3)];
    
% Damping Matrix
C_mat = [0,0;
        0,0];
    
% Stiffness Matrix
K_mat = [0,0;
        0,-(m*g*l)/2];
    
% State Weighing Matrix
A = [zeros(n,n),eye(n,n);
     -M_mat\K_mat,-M_mat\C_mat];
 
B = [zeros(n,n);M_mat\eye(size(M_mat))];

% Desired Positions
qd = wd(1:2);
qd_dot = wd(3:4);

% Generalized Coordinates
q = [w(1);w(2)];

% Generalized Velocities
q_dot = [w(3);w(4)];

% Disturbance Forces
switch disturbance
    case 'None'
        fdist = [0;0];
    case 'Impulse'
        if t >= 5 && t <= 5 + (1/100)
%             fdist = [0;2];
            fdist = [100;0];
        else 
            fdist = [0;0];
        end
    case 'Harmonic'
%         fdist = [0;2*sin(t)];
        fdist = [2*sin(t);0];
    case 'Static'   
%         fdist = [0;2];
        fdist = [2;0];
    case 'Ramp'
        fdist = [2*t;0];
end

% Closed Loop System Input
% Only the Rotary Arm is actuated
% Disturbance Force is also accounted
u = [Kp*(qd - q) + Kd*(qd_dot - q_dot);0] + fdist;

% State Space Model
dwdt = A*w + B*u;
end