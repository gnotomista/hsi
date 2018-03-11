function [v,mu,varargout] = grasp_polygon(graph_topology_mat_file_name,varargin)

[Lapl,topological_order] = build_laplacian(graph_topology_mat_file_name);
N_robots = size(Lapl,1);

p = inputParser;
addOptional(p,'DT',0.01)
addOptional(p,'N_DATA',100)
addOptional(p,'PLOT_STUFF',false)
p.parse(varargin{:})

DT = p.Results.DT;
N_DATA = p.Results.N_DATA;
PLOT_STUFF = p.Results.PLOT_STUFF;

% find number of 'generalized joints' (based on the defined topology)
idcs_all = find(Lapl==-1);
idcs = [];
for i = 1 : length(idcs_all)
    [I,J] = ind2sub(size(Lapl),idcs_all(i));
    if J > I
        idcs(end+1) = idcs_all(i);
    end
end
n_joints = length(idcs);

% initialize loop variables
data_for_pca = double.empty(n_joints,0);

% simulate graspings to collect data
for g = 1 : N_DATA
    
    clc
    fprintf('grasping  # %d\n',g)
    
    % generate polygonal object to grasp
    N_verteces = 5 + randi(20);
    x = rand(N_verteces,1);
    y = rand(N_verteces,1);
    th = atan2(y-mean(y), x-mean(x));
    [~, idx_o] = sort(th);
    xo = x(idx_o);
    yo = y(idx_o);
    % idx_c = convhull(x,y);
    % xc = x(idx_c);
    % yc = y(idx_c);
    
    % calculate centroid (and center) of the object
    P = [xo' xo(1); yo' yo(1)];
    M = [mean(x); mean(y)];
    G = centroid(P);
    
    % arrange robot on a circle centered at the object centroid
    r = 1.5*max(sqrt(sum((P-G).^2,1)));
    p0 = G + r*[cos(linspace(0,N_robots/(N_robots+1)*2*pi,N_robots));
        sin(linspace(0,N_robots/(N_robots+1)*2*pi,N_robots))];
    
    % plot stuff %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if PLOT_STUFF
        close all
        figure
        axis equal
        axis([G(1)-1.1*r G(1)+1.1*r G(2)-1.1*r G(2)+1.1*r])
        hold on
        plot(x,y,'o')
        hO = plot([xo; xo(1)], [yo; yo(1)]);
        % plot(xc, yc);
        plot(M(1), M(2), 'k+')
        plot(G(1), G(2), 'd')
        hR = plot(p0(1,:), p0(2,:), 'x');
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % move robots towards object centroid with a proportional controller
    p = p0;
    in = zeros(N_robots,1);
    while true
        % move robots %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for i = 1 : N_robots
            if ~in(i)
                in(i) = inpolygon(p(1,i), p(2,i), P(1,:), P(2,:));
                p(:,i) = p(:,i) + (G-p(:,i))*DT;
            end
        end
        if all(in)
            break
        end
        % plot stuff %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if PLOT_STUFF
            set(hR, 'XData', p(1,:), 'YData', p(2,:))
            if all(in)
                set(hO, 'LineWidth', 2)
            end
            drawnow limitrate
            pause(0.001)
        end        
        % record data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for i = 1 : N_robots
            neighbors = topological_neighbors(Lapl, i);
            for j = neighbors
                if j > i
                    if i == 1
                        data_for_pca(get_index(topological_order,[i,j]),end+1*(j == neighbors(1))) = norm(p(:,i)-p(:,j));
                    else
                        data_for_pca(get_index(topological_order,[i,j]),end) = norm(p(:,i)-p(:,j));
                    end
                end
                % data_for_pca
                % pause(0.01)
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    
    if PLOT_STUFF
        pause(0.5)
    end
end

[v,~,~,~,~,mu] = pca(data_for_pca');
if nargout > 2
    varargout{1} = data_for_pca;
    if nargout > 3
        varargout{2} = Lapl;
    end
end

end