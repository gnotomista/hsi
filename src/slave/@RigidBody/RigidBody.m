classdef RigidBody < handle
    %OBJECT Container of an object with collision testing features
    %   _
    
    properties
        shape_
        o_ = eye(3);
    end % public properties
    
    properties (Access=private)
        
    end % private properties
    
    methods
        function this = RigidBody(V)
            this.shape_ = polyshape(V(:,1)', V(:,2)');
            [this.o_(1,3) this.o_(2,3)] = centroid(this.shape_);
            gcf; plot(this.shape_);
            
        end % Object (constructor)
        
        function [robot, minDist, edge, u, v] = check_collision(this, P)
            robot = [];
            minDist = [];
            u = [];
            v = [];
            edge = [];
            IN = inpolygon(P(:,1), P(:,2), this.shape_.Vertices(:,1), this.shape_.Vertices(:,2));
            for i = 1 : size(IN)
                if (IN(i) == 1)
                    D = [];
                    U = [];
                    V = [];
                    robot = [robot,i];
                    for j = 1 : size(this.shape_.Vertices)-1
                        [d, p1, p2] = minimumDistance(P(i,:), this.shape_.Vertices(j,:), this.shape_.Vertices(j+1,:));
                        D = [D, d];
                        U = [U, p1];
                        V = [V, p2];
                    end
                    [d, p1, p2] = minimumDistance(P(i,:), this.shape_.Vertices(end,:), this.shape_.Vertices(1,:));
                    D = [D, d];
                    U = [U, p1];
                    V = [V, p2];
                    [dist,index] = min(D);
                    minDist = [minDist, dist];
                    u = [u, U(index)];
                    v = [v, V(index)];
                    if (index <= size(D,2)-1)
                        E = [this.shape_.Vertices(index,:), this.shape_.Vertices(index+1,:)];
                    else
                        E = [this.shape_.Vertices(end,:), this.shape_.Vertices(1,:)];
                    end
                    edge = [edge; E];
                else
                    
                end
            end
        end % test_collision
    end % public methods
    
    methods (Access=private)
        
    end % private methods
    
end % class

