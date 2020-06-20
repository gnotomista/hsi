classdef RigidBody < matlab.mixin.Copyable
    %OBJECT Container of an object with collision testing features
    %   _
    
    properties
        shape_              % object shape
        vertices_b_         % vertices in body frame
        o_ = eye(3);        % center of mass pose
        mass_ = 5e-1;          % mass
        I_ = 5e-1;             % body inertia
        contacts_ = [];     % list of contacts in body frame
        Mb_ = zeros(3,3);   % body mass matrix
        G_ = [];            % grasp matrix
        t_ = [];            % body twist
        a_ = [];            % body acceleration
        Fb_ = [];           % body wrench
        dt_ = 0.01;         % integration time step
        b_ = 10;            % damping coefficient
        kp_ = 10;           % force gain
        
        % handles
        h_contact_points_
        h_shape_
    end % public properties
    
    properties (Access=private)
        
    end % private properties
    
    methods
        function this = RigidBody(V)
            this.shape_ = polyshape(V(:,1)', V(:,2)');
            [this.o_(1,3), this.o_(2,3)] = centroid(this.shape_);
            gcf; this.h_shape_ = plot(this.shape_, 'FaceColor', [0 0.4470 0.7410], 'FaceAlpha', 0.3500);
            this.Mb_ = [this.mass_*eye(2), zeros(2,1); zeros(1,2), this.I_];
            this.vertices_b_ = this.o_(1:2,1:2)'*(this.shape_.Vertices'-this.o_(1:2,3));
            this.t_ = zeros(3,1); this.a_ = zeros(3,1);
        end % Object (constructor)
        
        function [robot_idcs, contact_points, forces] = check_contact(this, P, h_fig)
            forces = [];
            robot_idcs = inpolygon(P(:,1), P(:,2), this.shape_.Vertices(:,1), this.shape_.Vertices(:,2));
            for i = 1 : size(robot_idcs)
                if (robot_idcs(i) == 1)
                    D = [];
                    U = [];
                    for j = 1 : size(this.shape_.Vertices)-1
                        [d, u, ~] = minimumDistance(P(i,:), this.shape_.Vertices(j,:), this.shape_.Vertices(j+1,:));
                        D = [D, d];
                        U = [U, u];
                    end
                    [d, u, ~] = minimumDistance(P(i,:), this.shape_.Vertices(end,:), this.shape_.Vertices(1,:));
                    D = [D, d];
                    U = [U, u];
                    
                    [~, index] = min(D);
                    
                    if (index <= size(D,2)-1)
                        E = [this.shape_.Vertices(index,:), this.shape_.Vertices(index+1,:)];
                    else
                        E = [this.shape_.Vertices(end,:), this.shape_.Vertices(1,:)];
                    end
                    contact_points(:,i) = E(1:2) + U(index)*(E(3:4)-E(1:2));
                else
                    contact_points(:,i) = P(i,1:2)';
                end
            end
            
            % draw contacts
            h_fig;
            hold on;
            if (robot_idcs(robot_idcs>0))
                delete(this.h_contact_points_);
                this.h_contact_points_ = plot(contact_points(1, find(robot_idcs)), contact_points(2, find(robot_idcs)), 'b*');
            end

        end % check_contact
        
        function C = step(this, robot_idcs, robot_cmd_velocity, friction, noise)
            this.b_ = friction;
            if(find(robot_idcs))

                % rotate robot velocity in contact frame
                contact_velocity = this.o_(1:2,1:2)'*robot_cmd_velocity; % assumption: contact = body orientation
                
                % compute body velocity
                this.a_ = this.Mb_\(this.G_*reshape(contact_velocity, size(this.G_,2),1)*this.kp_ - this.b_*this.t_);
                this.t_ = this.t_ + this.a_*this.dt_;
                
                v = this.t_(1:2);
                w = this.t_(3);
                
                % integrate
                theta = w*this.dt_;
                % Ro = rotz(theta); % ua 'o Phased Array System Toolbox pe 'na matric 'e rotazion?
                % e = [Ro(1:2,1:2), v*this.dt_; 0 0 1];
                e = [[cos(theta) -sin(theta); sin(theta) cos(theta)], v*this.dt_; 0 0 1];
                this.o_ = this.o_*e;
                
                % re-compute contact points after integration
                C = this.o_(1:2, 3) + this.o_(1:2,1:2)*this.contacts_;

                % re-draw vertices
                this.shape_.Vertices = (this.o_(1:2, 3)+this.o_(1:2,1:2)*this.vertices_b_)';
                gcf; delete(this.h_shape_); this.h_shape_ = plot(this.shape_, 'FaceColor', [0 0.4470 0.7410], 'FaceAlpha', 0.3500);
            else
                C = [];
            end
        end % step
        
        % getter setter methods
        function set_grasp_matrix(this, G)
            this.G_ = G;
        end
        
        function set_contact_points(this, contacts)
            if(~isempty(contacts))
                this.contacts_ = this.o_(1:2,1:2)'*(contacts-this.o_(1:2,3));
            end
        end
        
    end % public methods
    
    methods (Access=private)
        
    end % private methods
    
end % class

