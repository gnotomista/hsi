clc
clear
close all

plot_stuff = true;
dt = 0.01;
N_robots = 12;
data_for_pca = [];

for g = 1 : 100
    
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
    
    % move robots towards object centroid
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot stuff
    if plot_stuff
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
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    p = p0;
    in = zeros(N_robots,1);
    out = zeros(N_robots,1);
    state = 'grasp';
    while true
        switch state
            case 'grasp'
                for i = 1 : N_robots
                    if ~in(i)
                        in(i) = inpolygon(p(1,i), p(2,i), P(1,:), P(2,:));
                        p(:,i) = p(:,i) + (G-p(:,i))*dt;
                    end
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% record data
                for i = 1 : N_robots
                    if i == 1
                        data_for_pca(i,end+1) = norm(p(:,i)-p(:,end));
                    else
                        data_for_pca(i,end) = norm(p(:,i)-p(:,i-1));
                    end
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if all(in)
                    % state = 'release';
                    break % break instead, no need to release
                end
            case 'release'
                for i = 1 : N_robots
                    if ~out(i)
                        out(i) = (norm(p(:,i)-p0(:,i)) < 2e-3);
                        p(:,i) = p(:,i) + (p0(:,i)-p(:,i))*dt;
                    end
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% record data
                for i = 2 : N_robots
                    if i == 1
                        data_for_pca(i,end+1) = norm(p(:,i)-p(:,end));
                    else
                        data_for_pca(i,end) = norm(p(:,i)-p(:,i-1));
                    end
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if all(out)
                    break
                end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot stuff
        if plot_stuff
            set(hR, 'XData', p(1,:), 'YData', p(2,:))
            if all(in)
                set(hO, 'LineWidth', 2)
            end
            drawnow limitrate
            pause(0.001)
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    
    if plot_stuff
        pause(0.5)
    end
end

v = pca(data_for_pca');

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