%**************************************************************************
%*********** FUNCTION TO CALCULATE THE ARM MATRIX (DISTAL) ****************
%**************************************************************************

function [A] = ArmMatrix_dist(A,sim_time,theta,link_lengths,NJ)


% INPUT VARIBLES
% Link and linear joint variables
alpha = zeros(NJ,1);
d = zeros(NJ,1);
a = zeros(NJ,1);
alpha(1) = (90)*(pi/180);
d(2) = link_lengths(1);
a(2) = link_lengths(2);


% alpha = zeros(NJ,1);    % alpha(k)  =   Twist Angle (Angle b/w Z(k-1) & Z(k) along X(k))
% a = zeros(NJ,1);        % a(k)      =   Link Length (Distance from Z(k-1) to Z(k) along X(k))
% theta = zeros(NJ,1);    % theta(k)  =   Joint Angle (Angle b/w X(k-1) & X(k) along Z(k-1)
% d = zeros(NJ,1);        % d(k)      =   Joint Offset(Distance from X(k-1) to X(k) along Z(k-1)
% flag = zeros(NJ,1);     % flag(k)   =   Decision about the Joint Type (Rotational(flag=1) or Prismatic (flag=0)) 
% dQ_dt = zeros(NJ,1);    % dQ_dt(k)  =   Actuator Velocity

for ii = 1:length(sim_time(1:end))
    for k = 1:NJ
        
        A{ii}(:,:,k) =   [ cos(theta{ii}(k))   -cos(alpha(k))*sin(theta{ii}(k))     sin(alpha(k))*sin(theta{ii}(k))     a(k)*cos(theta{ii}(k)); 
                           sin(theta{ii}(k))    cos(alpha(k))*cos(theta{ii}(k))    -sin(alpha(k))*cos(theta{ii}(k))     a(k)*sin(theta{ii}(k)); 
                            0                   sin(alpha(k))                         cos(alpha(k))                        d(k);   
                            0                   0                                      0                                     1 ]; 
    end
end

    
    


