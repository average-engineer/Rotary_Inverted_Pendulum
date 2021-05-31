function [J] = inertia_matrix_dist(link_lengths,m,r,NJ)
%units: kgm2
%ASSUMPTIONS
%the inertias of each segment along the X axis of its distal joint frame is assumed
%to be negligible
%the inertias of each segment along the Y and Z axes of its distal frame are
%assumed to be equal
%cross inertias (Ixx,Ixy,Ixz) are assumed to be zero

%inertias of link i about the frame of distal joint (ith joint)
Ixx = zeros(NJ,1);
Iyy = zeros(NJ,1);
Izz = zeros(NJ,1);
Ixy = zeros(NJ,1);
Iyz = zeros(NJ,1);
Ixz = zeros(NJ,1);

Ixx(1) = m(1)*(link_lengths(1)^2)/3;
Iyy(1) = Ixx(1);

Iyy(2) = m(2)*(link_lengths(2)^2)/3;
Izz(2) = Iyy(2);

for i = 1:NJ
    %inertia matrix for each link i
    J{i} = [(-Ixx(i) + Iyy(i) + Izz(i))/2,Ixy(i),Ixz(i),m(i)*r{i}(1);
        Ixy(i),(Ixx(i) - Iyy(i) + Izz(i))/2,Iyz(i),m(i)*r{i}(2);
        Ixz(i),Iyz(i),(Ixx(i) + Iyy(i) - Izz(i))/2,m(i)*r{i}(3);
        m(i)*r{i}(1),m(i)*r{i}(2),m(i)*r{i}(3),m(i)];
end
end