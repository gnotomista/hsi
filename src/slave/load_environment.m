V = {[ 2 -2.5;  2 3; -0 3; -0 2.5; 1.5 2.5; 1.5 -2.5], ... % obstacles
    [ 5 5; 5 0; 4.5 0; 4.5 5]} ;

angle = pi/2;
G = [cos(angle), -sin(angle); sin(angle), cos(angle)]*[-.3 -0.3; -.3 0.3; 0.1 0.3; 0.3 -.2]' + [6 4]';

c_obs   = [1 0.4470 0.7410];                             % color
c_goal  = [0.4470 1 0.7410];                             % color

for i = 1:length(V)
    p = polyshape(V{i}(:,1)', V{i}(:,2)');
    gcf; hold on;
    plot(p, 'FaceColor', c_obs);
end

p = polyshape(G(1,:), G(2,:));
gcf; hold on;
plot(p, 'FaceColor', c_goal);

axis equal