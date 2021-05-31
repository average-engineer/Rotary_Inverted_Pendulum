function [U] = U_matrix(A,sim_time,NJ,flag)

if flag == 1
    %Q matrix for revolute joint
    Q = [0,-1,0,0;1,0,0,0;0,0,0,0;0,0,0,0];
else
    Q = [zeros(1,4);zeros(1,4);0,0,0,1;zeros(1,4)];
end

for ii = 1:length(sim_time(1:end))
    for i = 1:NJ
        for j = 1:NJ
            if j>i
                U{ii}{i,j} = zeros(4,4);
            else
                T1 = T_Concat_dist(A{ii},j-1,0);
                T2 = T_Concat_dist(A{ii},i,j-1);
                U{ii}{i,j} = T1*Q*T2;
            end
        end
    end
end
end