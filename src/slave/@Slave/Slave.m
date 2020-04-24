classdef Slave < handle
    %SLAVE Models the slave of a synergy-based grasping architecture by a multi-robot system
    %   _
    
    properties
        N
        L
        to
        robot_poses
        pc
        mu
        robotarium_container
        robots_in_contact
        G
    end % public properties
    
    properties (Access=private)
        weights_mean
        weights
        EPS_DX
        FORMATION_CONTROL_GAIN
        MAX_ITER
    end % private properties
    
    methods
        function this = Slave(all_mat_file_name)
            this.load_all(all_mat_file_name);
            this.initialize_robotarium();
            this.build_up_weights();
                        
            this.EPS_DX = 0.05;
            this.FORMATION_CONTROL_GAIN = 10;
            this.MAX_ITER = 200;
            this.robots_in_contact = [];
            this.G = zeros(3, 2*this.N); 
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
                            this.FORMATION_CONTROL_GAIN * (norm(x(1:2, i) - x(1:2, j))^2 - (this.weights_mean(i, j) + syn_val*this.weights{syn_id}(i, j))^2) * (x(1:2, j) - x(1:2, i));
                    end
                end
                
                du = this.robotarium_container.si_to_uni_dyn(dx, x);
                
                this.robotarium_container.r.set_velocities(1:this.N, du);
                this.robotarium_container.r.step();
            end
            this.robot_poses = x;
        end % move_synergy
        
        function J = get_jacobian(this)
            assert(~isempty(this.robot_poses),'Robot poses is empty. Make sure to call move_synergy before calling get_jacobian.')
            J = zeros(2*this.N, length(this.to));
            for n = 1 : 2 : size(J,1)
                for k = 1 : size(J,2)
                    ij = this.to{k};
                    J(n,k) = this.robot_poses(1,ij(1)) - this.robot_poses(1,ij(2));
                    J(n+1,k) = this.robot_poses(2,ij(1)) - this.robot_poses(2,ij(2));
                end
            end
        end % get_jacobian
        
        function A = get_A(this)
            % A: see "Prattihizzo - Mapping syergies - TRO"
            assert(~isempty(this.robot_poses),'Robot poses is empty. Make sure to call move_synergy before calling get_jacobian.')
            o = mean(this.robot_poses(1:2,:),2);
            M = [0 -1; 1 0];
            A = zeros(2*this.N, 5);
            for n = 1 : this.N
                p = this.robot_poses(1:2,n);
                A(2*n-1:2*n,1:2) = eye(2);
                A(2*n-1:2*n,3:4) = norm(p-o)*M;
                A(2*n-1:2*n,5) = p-o;
            end
        end % get_A
    end % public methods
    
    methods (Access=private)
        function load_all(this,all_mat_file_name)
            load(all_mat_file_name)
            this.N = size(Lapl,1);
            this.L = Lapl;
            this.to = topological_order;
            this.pc = synergies_vectors;
            this.mu = synergies_mean';
        end % load_all
        
        function build_up_weights(this)
            % build up weight matrix for formation control
            this.weights = cell(size(this.pc,2),1);
            this.weights_mean = NaN(size(this.L));
            for ww = 1 : length(this.weights)
                this.weights{ww} = zeros(this.N);
                for i = 1 : size(this.L,1)
                    for j = 1 : size(this.L,2)
                        if this.L(i,j) == -1
                            if j > i
                                this.weights_mean(i,j) = this.mu(get_index(this.to,[i,j]));
                                this.weights{ww}(i,j) = this.pc(get_index(this.to,[i,j]),ww);
                            else
                                this.weights_mean(i,j) = this.mu(get_index(this.to,[j,i]));
                                this.weights{ww}(i,j) = this.pc(get_index(this.to,[j,i]),ww);
                            end
                        end
                    end
                end
            end
        end % build_up_weights
        
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

