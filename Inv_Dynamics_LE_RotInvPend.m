function [joint_torque] = Inv_Dynamics_LE_RotInvPend(w,NJ,link_lengths,link_masses,sim_time,g)

%%
%**************************************************************************
%************ INVERSE DYNAMICS of ROTARY INVERTED PENDULUM ****************
%**************************************************************************
% WHERE
% NJ = Number of Joints
% NF = Number of Frames
% flag = Decision about the Joint Type
% For Rotary Joint (Flag=1),
% For Prismatic Joint (Flag=0)
% w: known states (generalized coordinates and velocities)
%**************************************************************************

%% SYSTEM PARAMETERS
%position of COM wrt proximal frames
for i = 1:NJ
    COM_prox(i) = 0.5;
end

%position of link i COM wrt to ith frame (frame of distal joint of link i) (right)
r{1} = [0;0;link_lengths(1)/2;1];
r{2} = [-link_lengths(2)/2;0;0;1];
    
% Types of joints : Revolute
flag = ones(NJ,1);

%% DISTAL DH PARAMETERS
%joint variables (according to distal DH parameters of the system)
%radians
%according to distal DH parameters, there will be 2 joint variables

% theta1 : Rotary arm angular displacement
% theta2 : Inverted Pendulum angular displacement

for ii = 1:length(sim_time)
    for i = 1:NJ
        % Joint Variables (Generalized Coordinates)
        theta{ii}(i) = (pi/2) + w(ii,i);
        % Joint Velocities
        theta_dot{ii}(i) = w(ii,i + 2);
    end
end

% using finite difference method of derivation to compute accelerations
theta_dot_dot = finite_diff_vector(sim_time,w(:,3:4));
   
%Acceleration due to gravity matrix
gravity_acc = zeros(1,4);
gravity_acc(3) = -g;

%MEMORY ALLOCATION
for i = 1:length(sim_time)
    A{i} = zeros(4,4,NJ);% arm matrix
end

%% FUNCTION CALLS
% Calculate homogeneous tranforms of each frame
[A] = ArmMatrix_dist(A,sim_time,theta,link_lengths,NJ);

%inertia matrix/tensor for each link wrt to its distal joint frame
J = inertia_matrix_dist(link_lengths,link_masses,r,NJ);

%matrices for effect of the movement of one joint on other joint (Uij)
U = U_matrix(A,sim_time,NJ,flag);

%matrices for effect of the movement of two joints on another joint (Uijk)
U1 = U1_matrix(A,sim_time,NJ,flag);


%dynamic coeffient matrices
%inertial acceleration based matrix
D = inertia_acceleration_matrix(sim_time,U,J,NJ);
%coroilis and centrifugal force matrix
h = centri_cor_force_matrix(sim_time,U,U1,J,NJ,theta_dot);
%gravity loading based matrix
c = gravity_loading_matrix(sim_time,link_masses,gravity_acc,U,r,NJ);


%generalized torque matrix at each time instant
for ii = 1:length(sim_time)
    joint_torque(ii,:) = D{ii}*theta_dot_dot{ii} + h{ii} + c{ii};
%     joint_torque(ii) = -joint_torques{ii};
end

end