clc
clear
close all

% f_vec = 2xN-matrix
% r_vec = application points of the forces f_vec
% (for convenience, it is the same used to evaluate the normal component of the force)

global N m mu_vec
N = 4;
m = 1;
mu_vec = ones(N,1);

X0 = [-[1 0 -1 0; 0 1 0 -1], [1 0 -1 0; 0 1 0 -1]];
plot_forces(X0)
pause(1)

opts = optimoptions(@fmincon,'MaxFunctionEvaluations',1e6);

X = fmincon(@f_cost,X0,[],[],[],[],[],[],@nlc,opts);
plot_forces(X)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function J = f_cost(X)

d = size(X,2)/2;
f_vec = X(:,1:d);

J = norm(reshape(f_vec,numel(f_vec),1))^2;

end

function [c, ceq] = nlc(X)

global N m mu_vec

f_vec = X(:,1:N);
r_vec = X(:,N+1:end);

if size(f_vec,1) == 2
    f_vec = [f_vec; zeros(1,N)];
    r_vec = [r_vec; zeros(1,N)];
end

ceq = zeros(6+N,1);
% resultant force
ceq(1:3) = sum(f_vec,2);
% resultant couple
for i = 1 : N
    ceq(4:6) = ceq(4:6) + cross(r_vec(:,i), f_vec(:,i));
    ceq(6+i) = object_surface_constraint(r_vec(:,i));
end

% minimum force
c = m*9.81;
for i = 1 : N
    c = c - mu_vec(i)*abs(normal_component(f_vec(:,i), r_vec(:,i)));
end

end

function f_perp = normal_component(f, r)

% example for a ball-shaped body
n = -r;
f_perp = (f'*n)*n;

end

function s = object_surface_constraint(r)

% example for a ball-shaped body
s = 1-norm(r);

end

function plot_forces(X)

global N

f_vec = X(:,1:N);
r_vec = X(:,N+1:end);

if size(f_vec,1) == 2
    f_vec = [f_vec; zeros(1,N)];
    r_vec = [r_vec; zeros(1,N)];
end

figure
hold on
grid on
axis equal
plot3(cos(linspace(0,2*pi,100)),sin(linspace(0,2*pi,100)),zeros(100,1))
for i = 1 : N
    quiver3(r_vec(1,i),r_vec(2,i),r_vec(3,i),f_vec(1,i),f_vec(2,i),f_vec(3,i))
end

end




