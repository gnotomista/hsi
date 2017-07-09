clc
clear
close all

while true
    
    close all
    
    % generate polygonal object to grasp
    N_verteces = 10;
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
    N_robots = 12;
    r = 1.5*max(sqrt(sum((P-G).^2,1)));
    p0 = G + r*[cos(linspace(0,N_robots/(N_robots+1)*2*pi,N_robots));
        sin(linspace(0,N_robots/(N_robots+1)*2*pi,N_robots))];
    
    % move robots towards object centroid
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot stuff
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
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    dt = 0.001;
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
                if all(in)
                    state = 'release';
                end
            case 'release'
                for i = 1 : N_robots
                    if ~out(i)
                        out(i) = (norm(p(:,i)-p0(:,i)) < 2e-3);
                        p(:,i) = p(:,i) + (p0(:,i)-p(:,i))*dt;
                    end
                end
                if all(out)
                    break
                end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot stuff
        set(hR, 'XData', p(1,:), 'YData', p(2,:))
        if all(in)
            set(hO, 'LineWidth', 2)
        end
        drawnow limitrate
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    
    pause(0.5)
    
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