function F_Impulse = ImpulseForce(t,a,F,dt,n)

% Function for representing any arbrtitrary force as impulse force
% t: simulation time vector
% dt: time step size (seconds)
% a: time instant at which the impulse is experienced (seconds)
% F: the magnitude of the impulse felt
% The function is unit impulse function where the duration of the impulse
% force is the inverse of the impulse force magnitude
% n: number of independent DOFs of the system

% The element in time vector corresponding to start of impulse (a)
for i = 1:length(t)
    if t(i) == a
        inst = i;
        break;
    end
end 

% Duration of the disturbance
dur = 1/F;

% Number of time steps for which impulse load is present
dur_num = ceil(dur/dt);

for ii = 1:length(t)    
    F_Impulse(:,ii) = zeros(n,1);
    if ii >= inst && ii <= inst + dur_num
        F_Impulse(1,ii) = F; % 1st joint in the system experiences the impulse load
    end
end

end