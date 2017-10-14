classdef Slave
    %SLAVE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        N
        L
        robotarium_container
        pc
        weights
        EPS_DX
        FORMATION_CONTROL_GAIN
        MAX_ITER
    end
    
    methods
        function this = Slave(N, L)
            assert(size(L,1)==size(L,2), 'Laplacian must be square')
            assert(N==size(L,1), 'Number of robots N must equal dimension of Laplacian')
            this.N = N;
            this.L = L;
            this.robotarium_container = initialize_robotarium(N, L);
            this.EPS_DX = 0.05;
            this.FORMATION_CONTROL_GAIN = 10;
            this.MAX_ITER = 1000;
            
            % load synergies
            % grasping
            load('pc_grasping_slave.mat')
            this.pc(:,1) = v(:,1);
            % rotating
            % load('pc_rotating.mat')
            % this.pc(:,2) = v(:,2);
            
            % build up weight matrix for formation control
            this.weights = cell(size(this.pc,2),1);
            for ww = 1 : length(this.weights)
                this.weights{ww} = zeros(N);
                for i = 1 : size(L,1)
                    for j = 1 : size(L,2)
                        if L(i,j) == -1
                            if j > i
                                this.weights{ww}(i,j) = this.pc(this.robotarium_container.idcs==sub2ind(size(L),i,j));
                            else
                                this.weights{ww}(i,j) = this.pc(this.robotarium_container.idcs==sub2ind(size(L),j,i));
                            end
                        end
                    end
                end
            end
        end
        function move_synergy(this, syn_id, syn_val)
            dx = ones(2,this.N);
            iter = 0;
            while any(diag(dx'*dx) > this.EPS_DX) && iter < this.MAX_ITER
                iter = iter + 1;
                x = this.robotarium_container.r.get_poses();
                
                for i = 1 : this.N
                    dx(:, i) = zeros(2,1);
                    for j = topological_neighbors(this.L, i)
                        dx(:, i) = dx(:, i) + ...
                            this.FORMATION_CONTROL_GAIN * (norm(x(1:2, i) - x(1:2, j))^2 - syn_val*this.weights{syn_id}(i, j)^2) * (x(1:2, j) - x(1:2, i));
                    end
                end
                
                dx = this.robotarium_container.si_to_uni_dyn(dx, x);
                
                this.robotarium_container.r.set_velocities(1:this.N, dx);
                this.robotarium_container.r.step();
            end
        end
    end
    
end

