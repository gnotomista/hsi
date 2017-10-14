function visualize_synergy( robotarium, v, i )

% define some constants
FORMATION_CONTROL_GAIN = 10;

% retrieve stuffs
r = robotarium.r;
si_barrier_cert = robotarium.si_barrier_cert;
si_to_uni_dyn = robotarium.si_to_uni_dyn;
L = robotarium.L;
idcs = robotarium.idcs;

% get number of agents
N = r.get_number_of_agents();

% sanity check (necessary, but not sufficient)
assert(max(robotarium.idcs) < N^2, 'Number of robots specified larger than the one used during training')

% extract principal component to show
pc = v(:,i);

% build up weight matrix for formation control
weights = zeros(N);
for i = 1 : size(L,1)
    for j = 1 : size(L,2)
        if L(i,j) == -1
            if j > i
                size(pc)
                find(idcs==sub2ind(size(L),i,j))
                weights(i,j) = pc(idcs==sub2ind(size(L),i,j));
            else
                weights(i,j) = pc(idcs==sub2ind(size(L),j,i));
            end
        end
    end
end

% simulation loop
disp('drive to initial conditions');
dx = Inf(2, N);
while any(sum(dx.^2,1)>0.05)
    
    x = r.get_poses();
    
    for i = 1:N
        dx(:, i) = zeros(2,1);
        for j = topological_neighbors(L, i)
            dx(:, i) = dx(:, i) + ...
                FORMATION_CONTROL_GAIN * (norm(x(1:2, i) - x(1:2, j))^2 - weights(i, j)^2) * (x(1:2, j) - x(1:2, i));
        end
    end
    
    % dx = si_barrier_cert(dx, x);
    dx = si_to_uni_dyn(dx, x);
    
    r.set_velocities(1:N, dx);
    r.step();
    
end

disp('starting principal component visualization')

iterations = 1000000;
for t = 0 : iterations
    
    x = r.get_poses();
    
    for i = 1:N
        dx(:, i) = zeros(2,1);
        for j = topological_neighbors(L, i)
            dx(:, i) = dx(:, i) + ...
                FORMATION_CONTROL_GAIN * (norm(x(1:2, i) - x(1:2, j))^2 - (1-cos(2*pi*0.001*t))/2*weights(i, j)^2) * (x(1:2, j) - x(1:2, i));
        end
    end
    
    % dx = si_barrier_cert(dx, x);
    dx = si_to_uni_dyn(dx, x);
    
    r.set_velocities(1:N, dx);
    r.step();
    
end

r.call_at_scripts_end();

end

