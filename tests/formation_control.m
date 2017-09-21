clc
clear
close all

N = 4;

rb = RobotariumBuilder();
r = rb.set_number_of_agents(N).set_save_data(false).build();

linearVelocityGain = 1; 
angularVelocityGain = pi/2;
formationControlGain = 4;

iterations = 1000;

L = diag(2*ones(N,1));
for i = 1 : size(L,1)
    for j = 1 : size(L,2)
        if i == mod(j-1,N) || i == mod(j+1,N) || j == mod(i-1,N) || j == mod(i+1,N)
            L(i,j) = -1;
        end
    end
end

d = 0.2;
% d_diag = sqrt(2)*d;

weights = zeros(N);
for i = 1 : size(L,1)
    for j = 1 : size(L,2)
        if L(i,j) == -1
            weights(i,j) = d;
        end
    end
end
    
dx = zeros(2, N);

si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 0.06);
si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', linearVelocityGain, ... 
    'AngularVelocityLimit', angularVelocityGain);

for t = 0:iterations
    
    x = r.get_poses();
    
    for i = 1:N
        dx(:, i) = zeros(2,1);        
        for j = topological_neighbors(L, i)
            dx(:, i) = dx(:, i) + ...
            formationControlGain*(norm(x(1:2, i) - x(1:2, j))^2 - weights(i, j)^2) ... 
            *(x(1:2, j) - x(1:2, i));
        end 
    end
    
    dx = si_barrier_cert(dx, x);
    dx = si_to_uni_dyn(dx, x);  
    
    r.set_velocities(1:N, dx);
    r.step();   
end

r.call_at_scripts_end();




