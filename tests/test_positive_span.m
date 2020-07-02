clc
clear
close all

% N = 6;
% t = linspace(0,2*pi*(N-1)/N,N);
% V = [cos(t);
%     sin(t)];
% b = -1+2*rand(2,1)

V = [-0.5931    0.0889    0.9972    0.2477   -0.4441   -0.9999
    -0.8051   -0.9960    0.0742    0.9688    0.8960   -0.0127
    -0.1145   -0.1439   -0.0029    0.0886    0.0561   -0.0340];
b = [1.0070
    -19.9746
    0];
N = size(V,2);

lambda = quadprog(eye(N),zeros(1,N),[],[],V,b,zeros(N,1),[])

V*lambda

[A, b, Aeq, beq] = vert2lcon(V');
(b<0)

positive_span_is_all(V)

% figure, hold on, axis equal, axis([-2 2 -2 2])
% scatter(V(1,:), V(2,:), 1000, '.')
% scatter(b(1), b(2), 1000, '.')

function psia = positive_span_is_all(A)
[~, b, ~, ~] = vert2lcon(A');
if all(b>0)
    psia = true;
else
    psia = false;
end
end