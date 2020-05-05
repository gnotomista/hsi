classdef Slave < handle
    %SLAVE Models the slave of a synergy-based grasping architecture by a multi-robot system
    %   _
    
    properties
        N
        L
        to
        robot_poses
        theta_hinge
        pc
        mu
        robotarium_container
        robots_in_contact_ids
        robots_in_contact_num
        G
        u
        v
    end % public properties
    
    properties (Access=private)
        weights_mean
        weights
        EPS_DX
        FORMATION_CONTROL_GAIN
        max_iter
        optimparam
    end % private properties
    
    methods
        function this = Slave(all_mat_file_name)
            this.load_all(all_mat_file_name);
            this.initialize_robotarium();
            this.robot_poses = this.robotarium_container.r.get_poses();
            this.robotarium_container.r.step();
            this.build_up_weights();
            this.theta_hinge = zeros(1,this.N);
            this.EPS_DX = 0.05;
            this.FORMATION_CONTROL_GAIN = 10;
            this.max_iter = 10;
            this.optimparam.optimoptions = optimoptions(@quadprog, 'Display', 'off');
            this.optimparam.kappa = 1;
            this.optimparam.H = diag([ones(1,this.N) this.optimparam.kappa*ones(1,this.N)]);
            this.optimparam.f = zeros(1,2*this.N);
            this.optimparam.theta_max = pi;
            this.optimparam.V = @(th, th_hinge) (th-th_hinge)^2;
            this.optimparam.h = @(th, th_hinge) this.optimparam.theta_max^2 - (th-th_hinge)^2;
            this.optimparam.dV_dth = @(th, th_hinge) 2*(th-th_hinge);
            this.optimparam.dh_dth = @(th, th_hinge) -2*(th-th_hinge);
            this.optimparam.alpha = @(s) s;
            this.G = [];
        end % Slave (constructor)
        
        function swarm_syn(this, syn_id, syn_val)
            dx = ones(2,this.N);
            for i = 1 : this.N
                dx(:, i) = zeros(2,1);
                for j = topological_neighbors(this.L, i)
                    dx(:, i) = dx(:, i) + ...
                        this.FORMATION_CONTROL_GAIN * (norm(this.robot_poses(1:2, i) - this.robot_poses(1:2, j))^2 - (this.weights_mean(i, j) + syn_val*this.weights{syn_id}(i, j))^2) * (this.robot_poses(1:2, j) - this.robot_poses(1:2, i));
                end
            end
            this.u = this.robotarium_container.si_to_uni_dyn(dx, this.robot_poses);
            theta = this.robot_poses(3,:);
            this.v = this.u(1,:).*[cos(theta); sin(theta)];
        end % swarm_syn
        
        function opt_obj_manip(this, v_obj_des, omega_obj_des)
            Theta = zeros(2*this.N, this.N);
            Aineq = zeros(2*this.N, 2*this.N); % (N CLFs + N CBFs) x (N v + N omega)
            bineq = zeros(2*this.N, 1);
            for i = 1 : this.N
                if ismember(i, this.robots_in_contact_ids)
                    thetai = this.robot_poses(3,i);
                    Theta(2*i-1:2*i, i) = [cos(thetai); sin(thetai)];
                    Aineq(i, this.N+i) = this.optimparam.dV_dth(thetai, this.theta_hinge(i));
                    bineq(i) = -this.optimparam.alpha(this.optimparam.V(thetai, this.theta_hinge(i)));
                    Aineq(this.N+i, this.N+i) = -this.optimparam.dh_dth(thetai, this.theta_hinge(i));
                    bineq(this.N+i) = this.optimparam.alpha(this.optimparam.h(thetai, this.theta_hinge(i)));
                end
            end
            Aeq = [this.G * Theta, zeros(3, this.N)];
            beq = [v_obj_des; omega_obj_des];
            % v_omega = quadprog(this.optimparam.H, this.optimparam.f, Aineq, bineq, Aeq, beq, [zeros(this.N,1); -inf(this.N,1)], inf(2*this.N,1), [], this.optimparam.optimoptions);
            % TODO: remove following line and uncomment the previous
            v_omega = quadprog(this.optimparam.H, this.optimparam.f, Aineq, bineq, Aeq, beq, [-inf(this.N,1); -inf(this.N,1)], inf(2*this.N,1), []);%, this.optimparam.optimoptions);
            this.u = reshape(v_omega',this.N,2)';
            theta = this.robot_poses(3,:);
            this.v = this.u(1,:).*[cos(theta); sin(theta)];
        end % opt_obj_manip
        
        function swarm_syn_and_opt_obj_manip(this, syn_id, syn_val, v_obj_des, omega_obj_des)
            this.swarm_syn(syn_id, syn_val);
            du_nom = this.u;
            if this.robots_in_contact_num > 0
                this.opt_obj_manip(v_obj_des, omega_obj_des);
                this.u(:,setdiff(1:this.N,this.robots_in_contact_ids)) = du_nom(:,setdiff(1:this.N,this.robots_in_contact_ids));
            end
        end % swarm_syn_and_opt_obj_manip
        
        function G = update_grasp_matrix(this, robot_idcs, contact_points, obj_centroid)
            % contact_points contains, on the columns, the poses of all the
            % robots, both if they are in contact with the object (in which
            % case the corresponding component of robot_idcs will be 1) and
            % if they are not.
            this.G = [];
            this.robots_in_contact_ids = find(robot_idcs);
            this.robots_in_contact_num = numel(this.robots_in_contact_ids);
            if this.robots_in_contact_num > 0
                this.G = zeros(3, 2*this.N);
                for i = 1 : this.N
                    if ismember(i, this.robots_in_contact_ids)
                        goc = eye(3);
                        goc(1:2,3) = obj_centroid(1:2, 1:2)'*(contact_points(:,i) - obj_centroid(1:2,3));
                        this.G(:, 2*i-1:2*i) = gi(inv(goc));
                    end
                end
            end
            G = this.G;
        end % update_grasp_matrix
        
        function x = retrieve_poses(this)
            this.robot_poses = this.robotarium_container.r.get_poses();
            x = this.robot_poses;
        end % retrieve_poses
        
        function overwrite_poses(this, x)
            this.robotarium_container.r.initialize(x);
        end % overwrite_poses
        
        function step(this)
            for i = 1 : this.max_iter
                try
                    this.robot_poses = this.robotarium_container.r.get_poses();
                catch
                end
                this.robotarium_container.r.set_velocities(1:this.N, this.u);
                this.robotarium_container.r.step();
            end
        end % step
        
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
        function set_theta_hinge(this, i, theta_hinge_i)
            this.theta_hinge(i) = theta_hinge_i;
        end % set_theta_hinge
        
        function set_kappa(this, kappa)
            this.optimparam.kappa = kappa;
        end % set_kappa
        
        function set_theta_max(this, theta_max)
            this.optimparam.theta_max = theta_max;
        end % set_theta_max
        
        function set_max_iter(this, max_iter)
            this.max_iter = max_iter;
        end
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

