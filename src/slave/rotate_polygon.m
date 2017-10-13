function [v,L] = rotate_polygon(n_robots,L,varargin)

p = inputParser;
addOptional(p,'DT',0.01)
addOptional(p,'N_GRASPING',100)
addOptional(p,'PLOT_STUFF',false)
p.parse(varargin{:})

DT = p.Results.DT;
N_GRASPING = p.Results.N_GRASPING;
PLOT_STUFF = p.Results.PLOT_STUFF;

% find number of 'generalized joints' (based on the defined topology)
idcs_all = find(L==-1);
idcs = [];
for i = 1 : length(idcs_all)
    [I,J] = ind2sub(size(L),idcs_all(i));
    if J > I
        idcs(end+1) = idcs_all(i);
    end
end
n_joints = length(idcs);

% initialize loop variables
data_for_pca = double.empty(n_joints,0);

% simulate graspings to collect data
for g = 1 : N_GRASPING
    
    clc
    fprintf('rotating  # %d\n',g)
    
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
    p0 = G + r*[cos(linspace(0,n_robots/(n_robots+1)*2*pi,n_robots));
        sin(linspace(0,n_robots/(n_robots+1)*2*pi,n_robots))];
    
    % plot stuff %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if PLOT_STUFF
        close all
        figure
        axis equal
        axis([G(1)-1.1*r G(1)+1.1*r G(2)-1.1*r G(2)+1.1*r])
        hold on
        hP = plot(x,y,'o');
        hO = plot([xo; xo(1)], [yo; yo(1)]);
        % plot(xc, yc);
        plot(M(1), M(2), 'k+')
        plot(G(1), G(2), 'd')
        hR = plot(p0(1,:), p0(2,:), 'x');
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % move robots towards object centroid with a proportional controller
    p = p0;
    in = zeros(n_robots,1);
    while true
        % move robots %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for i = 1 : n_robots
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
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    
    pt = zeros(size(p));
    for th = linspace(0, 2*pi, 360)
        Pt = G + rot(th) * (P - G);
        pt = G + rot(th) * (p - G);
        % record data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for i = 1 : n_robots
            neighbors = topological_neighbors(L, i);
            for j = neighbors
                if j > i
                    if i == 1
                        data_for_pca(idcs==sub2ind(size(L),i,j),end+1*(j == neighbors(1))) = norm(pt(:,i)-pt(:,end));
                    else
                        data_for_pca(idcs==sub2ind(size(L),i,j),end) = norm(pt(:,i)-pt(:,i-1));
                    end
                end
                % data_for_pca
                % pause(0.01)
            end
        end
        % plot stuff %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if PLOT_STUFF
            set(hR, 'XData', pt(1,:), 'YData', pt(2,:))
            set(hO, 'XData', Pt(1,:), 'YData', Pt(2,:))
            set(hP, 'XData', Pt(1,:), 'YData', Pt(2,:))
            drawnow limitrate
            pause(0.001)
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    
    if PLOT_STUFF
        pause(0.5)
    end
end

v = pca(data_for_pca');

end

function G = centroid(P)
n = size(P,2);
M = [0 1;-1 0];
A = 0;
S = 0;
for i = 1 : n
    ri = P(:,i);
    if i < n
        j = i + 1;
    else
        j = 1;
    end
    rj = P(:,j);
    rjo = M * rj;
    A = A + ri'*rjo;
    S = S + (ri' * rjo * (ri + rj));
end
A = A / 2;
S = S / 6;
G = S / A;
end

function R = rot(th)
R = [cos(th) -sin(th);
    sin(th) cos(th)];
end