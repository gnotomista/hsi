classdef Slave < handle
    %SLAVE Models the slave of a synergy-based grasping architecture by a multi-robot system
    %   _
    
    properties
        N
        L
        pc
        mu
        robotarium_container
    end % public properties
    
    properties (Access=private)
        idcs
        weights_mean
        weights
        EPS_DX
        FORMATION_CONTROL_GAIN
        MAX_ITER
    end % private properties
    
    methods
        function this = Slave()
            this.load_synergies();
            this.initialize_robotarium();
            this.build_up_weights();
            
            this.EPS_DX = 0.05;
            this.FORMATION_CONTROL_GAIN = 10;
            this.MAX_ITER = 200;
        end % Slave (constructor)
        
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
                            this.FORMATION_CONTROL_GAIN * (norm(x(1:2, i) - x(1:2, j))^2 - (this.weights_mean{syn_id}(i, j) + syn_val*this.weights{syn_id}(i, j))^2) * (x(1:2, j) - x(1:2, i));
                    end
                end
                
                du = this.robotarium_container.si_to_uni_dyn(dx, x);
                
                this.robotarium_container.r.set_velocities(1:this.N, du);
                this.robotarium_container.r.step();
            end
        end % move_synergy
    end % public methods
    
    methods (Access=private)
        function load_synergies(this)
            load('mat_files/synergies_grasp.mat')
            this.N = size(Lapl,1);
            this.L = Lapl;
            this.pc(:,1) = synergies_vectors(:,1);
            this.mu(:,1) = synergies_mean';
            
            load('mat_files/synergies_rotate.mat')
            assert(this.N==size(Lapl,1), 'Number of robots must be the same for grasping and rotation synergies')
            assert(all(size(this.L)==size(Lapl)), 'Laplacians must be the same for grasping and rotation synergies')
            this.pc(:,2) = synergies_vectors(:,1);
            this.mu(:,2) = synergies_mean';
            
            idcs_all = find(this.L==-1);
            this.idcs = [];
            for i = 1 : length(idcs_all)
                [I,J] = ind2sub(size(this.L),idcs_all(i));
                if J > I
                    this.idcs(end+1) = idcs_all(i);
                end
            end
        end % load_synergies
        
        function build_up_weights(this)
            % build up weight matrix for formation control
            this.weights = cell(size(this.pc,2),1);
            for ww = 1 : length(this.weights)
                this.weights{ww} = zeros(this.N);
                for i = 1 : size(this.L,1)
                    for j = 1 : size(this.L,2)
                        if this.L(i,j) == -1
                            if j > i
                                this.weights_mean{ww}(i,j) = this.mu(this.idcs==sub2ind(size(this.L),i,j),ww);
                                this.weights{ww}(i,j) = this.pc(this.idcs==sub2ind(size(this.L),i,j),ww);
                            else
                                this.weights_mean{ww}(i,j) = this.mu(this.idcs==sub2ind(size(this.L),j,i),ww);
                                this.weights{ww}(i,j) = this.pc(this.idcs==sub2ind(size(this.L),j,i),ww);
                            end
                        end
                    end
                end
            end
        end
        
        function initialize_robotarium(this)
            rb = RobotariumBuilder();
            this.robotarium_container.r = rb.set_number_of_agents(this.N).set_save_data(false).set_show_figure(true).build();
            
            linearVelocityGain = 1;
            angularVelocityGain = pi/2;
            
            this.robotarium_container.si_pos_ctrl = create_si_position_controller('XVelocityGain', 2, 'YVelocityGain', 2);
            this.robotarium_container.si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 0.06);
            this.robotarium_container.si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', linearVelocityGain, ...
                'AngularVelocityLimit', angularVelocityGain);
        end % initialize_robotarium
    end % private methods
    
end % class

