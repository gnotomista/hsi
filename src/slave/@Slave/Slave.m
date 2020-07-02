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
        bouding_box_radius
        h_bouding_box
    end % public properties
    
    properties %(Access=private) % TODO: commented during testing
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
            this.robots_in_contact_ids = [];
            this.robots_in_contact_num = 0;
            this.build_up_weights();
            this.theta_hinge = zeros(1,this.N);
            this.EPS_DX = 0.05;
            this.FORMATION_CONTROL_GAIN = 10;
            this.max_iter = 10;
            this.optimparam.optimoptions = optimoptions(@quadprog, 'Display', 'off');
            this.optimparam.kappa = 1;
            this.optimparam.H = diag([eye(1,this.N) this.optimparam.kappa*ones(1,this.N) 1e3]);
            this.optimparam.f = [zeros(1,2*this.N) 0];
            this.optimparam.theta_max = pi;
            this.optimparam.V = @(th, th_hinge) (th-th_hinge)^2;
            this.optimparam.h = @(th, th_hinge) this.optimparam.theta_max^2 - (th-th_hinge)^2;
            this.optimparam.dV_dth = @(th, th_hinge) 2*(th-th_hinge);
            this.optimparam.dh_dth = @(th, th_hinge) -2*(th-th_hinge);
            this.optimparam.alpha = @(s) s;
            this.G = [];
            this.bouding_box_radius = 0.08;
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
            % used the classic diffeo instead of: this.u = this.robotarium_container.si_to_uni_dyn(dx, this.robot_poses);
            for i = 1 : this.N
                this.u(:,i) = diag([1 1/0.01]) * [cos(this.robot_poses(3,i)) sin(this.robot_poses(3,i)); -sin(this.robot_poses(3,i)) cos(this.robot_poses(3,i))] * dx(:,i);
            end
            theta = this.robot_poses(3,:);
            this.v = this.u(1,:).*[cos(theta); sin(theta)];
        end % swarm_syn
        
        function exit_flag = opt_obj_manip(this, v_obj_des, omega_obj_des, Ro)
            Theta = zeros(2*this.N, this.N);
            Aineq = zeros(2*this.N+1, 2*this.N+1);
            bineq = zeros(2*this.N+1, 1);
            % build Theta matrix
            for i = 1 : this.N
                if ismember(i, this.robots_in_contact_ids)
                    thetai = this.robot_poses(3,i);
                    Theta(2*i-1:2*i, i) = Ro'*[cos(thetai); sin(thetai)];
                end
            end
            GT = this.G*Theta;
            % space_inside_polyhedron = this.score_manip(GT(:,this.robots_in_contact_ids)); % TODO test
            V = this.repulsive_potential(GT);
            for i = 1 : this.N
                if ismember(i, this.robots_in_contact_ids)
                    % hinge constraints
                    % Aineq(i, this.N+i) = this.optimparam.dV_dth(thetai, this.theta_hinge(i));
                    % bineq(i) = -this.optimparam.alpha(this.optimparam.V(thetai, this.theta_hinge(i)));
                    % Aineq(this.N+i, this.N+i) = -this.optimparam.dh_dth(thetai, this.theta_hinge(i));
                    % bineq(this.N+i) = this.optimparam.alpha(this.optimparam.h(thetai, this.theta_hinge(i)));
                    % manipulability constraints
                    dV_dthetai = 0;
                    thetai = this.robot_poses(3,i);
                    Thetai = Theta;
                    Thetai(2*i-1:2*i, i) = Ro'*[cos(thetai+1e-2); sin(thetai+1e-2)];
                    DeltaV = this.repulsive_potential(this.G*Thetai) - V;
                    DeltaThetai = 1e-2;
                    for j = i+1 : this.N
                        if ismember(j, this.robots_in_contact_ids)
                            dV_dthetai = dV_dthetai + DeltaV/DeltaThetai;
                        end
                    end
                    Aineq(2*this.N+1, this.N+i) = dV_dthetai;
                    bineq(2*this.N+1) = -10*this.optimparam.alpha(V);
                end
            end
            Aeq = [this.G * Theta, zeros(3, this.N), ones(3,1)];
            beq = [Ro'*v_obj_des; omega_obj_des];
            % v and omega together ...
            % v_omega = quadprog(this.optimparam.H, this.optimparam.f, Aineq, bineq, Aeq, beq, [zeros(this.N,1); -inf(this.N,1); -inf], inf(2*this.N+1,1), [], this.optimparam.optimoptions);
            % this.u = reshape(v_omega(1:2*this.N),this.N,2)';
            % ... or separately to otimize manipulability
            [input_v, ~, exit_flag] = quadprog(eye(this.N), zeros(1,this.N), [], [], Aeq(:,1:this.N), beq, zeros(this.N,1), [], [], this.optimparam.optimoptions);
            if exit_flag < 0
                input_v = zeros(this.N,1);
            end
            input_omega = quadprog(eye(this.N), zeros(1,this.N), Aineq(2*this.N+1,this.N+1:end-1), bineq(2*this.N+1), [], [], [], [], [], this.optimparam.optimoptions);
            this.u = [input_v';
                input_omega'];
            theta = this.robot_poses(3,:);
            this.v = this.u(1,:).*[cos(theta); sin(theta)];
        end % opt_obj_manip
        
        function exit_flag = swarm_syn_and_opt_obj_manip(this, syn_id, syn_val, v_obj_des, omega_obj_des, Ro)
            this.swarm_syn(syn_id, syn_val);
            du_nom = this.u;
            exit_flag = [];
            if this.robots_in_contact_num > 0
                exit_flag = this.opt_obj_manip(v_obj_des, omega_obj_des, Ro);
                % (a) robots in contact are not controlled by the synergies
                % this.u(:,setdiff(1:this.N,this.robots_in_contact_ids)) = du_nom(:,setdiff(1:this.N,this.robots_in_contact_ids));
                % (b) robots in contact are controlled also by the synergies
                % this.u = this.u + du_nom;
                this.u = this.u + 0.5*du_nom; % TODO: test weighted average
            end
            theta = this.robot_poses(3,:);
            this.v = this.u(1,:).*[cos(theta); sin(theta)];
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
                this.robot_poses = this.robotarium_container.r.get_poses();
            end
        end % step
        
        function plot_bb(this, c)
            delete(this.h_bouding_box);
            for i = 1 : this.N
                x = this.robot_poses(1, i);
                y = this.robot_poses(2, i);
                hold on
                th = 0:pi/50:2*pi;
                x_circle = this.bouding_box_radius * cos(th) + x;
                y_circle = this.bouding_box_radius * sin(th) + y;
                this.h_bouding_box = [this.h_bouding_box, plot(x_circle, y_circle)];
                this.h_bouding_box = [this.h_bouding_box, fill(x_circle, y_circle, c, 'FaceAlpha', 0.5)];
                hold off
            end
        end % plot_bb
        
        function rob_pos_coll = check_robot_collisions(this, rob_pos)
            rob_pos_coll = rob_pos;
            for i = 1 : this.N-1
                for j = i+1 : this.N
                    if(norm(rob_pos(1:2,i) - rob_pos(1:2,j)) < 2*this.bouding_box_radius)
                        coll_pen = norm(rob_pos(1:2,i) - rob_pos(1:2,j))-2*this.bouding_box_radius;
                        rob_pos_coll(1:2,i) = rob_pos_coll(1:2,i) - 0.5*coll_pen*(rob_pos(1:2,i) - rob_pos(1:2,j))/norm(rob_pos(1:2,i) - rob_pos(1:2,j));
                        rob_pos_coll(1:2,j) = rob_pos_coll(1:2,j) - 0.5*coll_pen*(rob_pos(1:2,j) - rob_pos(1:2,i))/norm(rob_pos(1:2,i) - rob_pos(1:2,j));
                    end
                end
            end
        end
        
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
        
        function GTh = get_grasp_theta(this, Ro)
            Theta = zeros(2*this.N, this.N);
            for i = 1 : this.N
                if ismember(i, this.robots_in_contact_ids)
                    thetai = this.robot_poses(3,i);
                    Theta(2*i-1:2*i, i) = Ro'*[cos(thetai); sin(thetai)];
                end
            end
            GTh = this.G * Theta;
        end % get_grasp_theta
        
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
            init_poses = [init_positions; rand(1,this.N).*atan2(-init_positions(2,:), -init_positions(1,:))];
            this.robotarium_container.r = Robotarium('NumberOfRobots', this.N, 'ShowFigure', true, 'InitialConditions', init_poses);
            this.robotarium_container.si_to_uni_dyn = create_si_to_uni_dynamics();
        end % initialize_robotarium
        
        function d = score_manip(this, p)
            m = size(p, 1);
            [A, b, Aeq, beq] = vert2lcon(p');
            closestpoint = lsqlin(speye(m), zeros(m,1), A, b, Aeq, beq, [], [], [], this.optimparam.optimoptions);
            d = norm(closestpoint);
            if all(b>0) % 0 is inside the polyhedron
                d = abs(d);
            else
                d = -abs(d);
            end
        end % score_manip
        
        function V = repulsive_potential(this, GT_contact)
            V = 0;
            for i = 1 : size(GT_contact,2)
                for j = i+1 : size(GT_contact,2)
                    V = V + 1/(norm(GT_contact(:,i)-GT_contact(:,j))+1);
                end
            end
        end
    end % private methods
    
end % class

