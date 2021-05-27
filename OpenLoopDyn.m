function [dwdt] = OpenLoopDyn(t,w,m,M,L,l,g)
% Function for modelling the non-linear open loop dynamics of Rotary
% inverted pendulum system

% Mass Matrix
M_mat = [(M*(L^2))/3 + m*(L^2) + (m*(l^2)*((sin(w(2)))^2))/4 , -(m*L*l*cos(w(2)))/2;
           -(m*L*l*cos(w(2)))/2,  (m*(l^2))/3];
       
% Damping Matrix
C_mat = [(m*(l^2)*sin(w(2))*cos(w(2))*w(4))/2 , (m*L*l*sin(w(2))*w(4))/2 ; 
            -(m*(l^2)*sin(w(2))*cos(w(2))*w(3))/4 , (m*L*l*sin(w(2))*w(3))/2];
        
% Stiffness Matrix
K_mat = [0 ; -(m*g*l*sin(w(2)))/2];

% Generalized Coordinates and velocity vector
q = [w(1);w(2)];
qdot = [w(3);w(4)];

dwdt = [qdot ; 
        -M_mat\C_mat*qdot - M_mat\K_mat];
end