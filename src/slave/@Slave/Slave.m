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
        optimparam
    end % private properties
    
    methods
        function this = Slave(all_mat_file_name)
            this.load_all(all_mat_file_name);
            this.initialize_robotarium();
            this.build_up_weights();
                        
            this.EPS_DX = 0.05;
            this.FORMATION_CONTROL_GAIN = 10;
            this.MAX_ITER = 200;
            this.optimparam.optimoptions = optimoptions(@quadprog, 'Display', 'off');
            this.optimparam.kappa = 1;
            this.optimparam.H = diag([ones(1,this.N) this.optimparam.kappa*ones(1,this.N)]);
            this.optimparam.f = zeros(1,2*this.N);
            this.optimparam.theta_max = -pi;
            this.optimparam.V = @(th, th_hinge) (th-th_hinge)^2;
            this.optimparam.h = @(th, th_hinge) this.optimparam.theta_max^2 - (th-th_hinge)^2;
            this.optimparam.dV_dth = @(th, th_hinge) 2*(th-th_hinge);
            this.optimparam.dh_dth = @(th, th_hinge) -2*(th-th_hinge);
            this.optimparam.alpha = @(s) s;
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
        
        function shared_obj_manipulation(this, v_obj_des, omega_obj_des)
            this.robot_poses = this.robotarium_container.r.get_poses();
            Theta = zeros(2*this.N,this.N);
            Aineq = zeros(2*this.N, 2*this.N); % (N CLFs + N CBFs) x (N v + N omega)
            bineq = zeros(2*this.N, 1);
            for i = 1 : this.N
                thetai = this.robot_poses(3,i);
                thetai_hinge = thetai; % TODO: this depends on the initial contact between the robots and the object
                Theta((i-1)*2+1:2*i, i) = [cos(thetai); sin(thetai)];
                Aineq(i, this.N+i) = this.optimparam.dV_dth(thetai, thetai_hinge);
                bineq(i) = -this.optimparam.alpha(this.optimparam.V(thetai, thetai_hinge));
                Aineq(this.N+i, this.N+i) = -this.optimparam.dh_dth(thetai, thetai_hinge);
                bineq(this.N+i) = this.optimparam.alpha(this.optimparam.h(thetai, thetai_hinge));
            end
            Aeq = [this.G * Theta, zeros(3, this.N)];
            beq = [v_obj_des; omega_obj_des];
            v_omega = quadprog(this.optimparam.H, this.optimparam.f, Aineq, bineq, Aeq, beq, [zeros(this.N,1); -inf(this.N,1)], inf(2*this.N,1), [], this.optimparam.optimoptions);
            du = reshape(v_omega',this.N,2)';
            this.robotarium_container.r.set_velocities(1:this.N, du);
            this.robotarium_container.r.step();
            % TODO: implement motion of the object and robot (ghosts and non-ghosts)
            %       also, now the poses of the robots are wrong, we need to
            %       consider the interaction with the object in simulation too
        end % shared_obj_manipulation
        
        % getters
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
        
        % setters
        function set_kappa(this, kappa)
            this.optimparam.kappa = kappa;
        end % set_kappa
        
        function set_theta_max(this, theta_max)
            this.optimparam.theta_max = theta_max;
        end % set_theta_max
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
            init_positions = [cos(2*pi/this.N:2*pi/this.N:2*pi); sin(2*pi/this.N:2*pi/this.N:2*pi)];
            init_poses = [init_positions; atan2(-init_positions(2,:), -init_positions(1,:))];
            this.robotarium_container.r = Robotarium('NumberOfRobots', this.N, 'ShowFigure', true, 'InitialConditions', init_poses);
            this.robotarium_container.si_to_uni_dyn = create_si_to_uni_dynamics();
        end % initialize_robotarium
    end % private methods
    
end % class

