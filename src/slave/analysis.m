% null-sapace analysis
clc
clear
close all

% define some constants
SYN_ID = 1; % 1: grasping, 2: rotation

s = Slave('mat_files/all_data');
for t = 0 : 100
    s.move_synergy(SYN_ID, (1-cos(2*pi*0.001*t))/2)
end

J = s.get_jacobian();

S = s.pc(:,1:2);

N = size(J,1)/2;
G_tilde = zeros(6,6*N);
for n = 1 : N
    G_tilde(1:3,3*(n-1)+(1:3)) = eye(3);
    G_tilde(4:6,3*(n-1)+(1:3)) = skew([s.robot_poses(:,n); 0]);
end
for n = N+1 : 2*N
    G_tilde(1:3,3*(n-1)+(1:3)) = zeros(3);
    G_tilde(4:6,3*(n-1)+(1:3)) = eye(3);
end

H = eye(3,6);

Jz = zeros(3/2*size(J,1),size(J,2));
for i = 1 : N
    Jz(3*(i-1)+1,:) = J(2*(i-1)+1,:);
    Jz(3*(i-1)+2,:) = J(2*(i-1)+2,:);
    Jz(3*(i-1)+3,:) = zeros(1,size(J,2));
end

ker([Jz*S -(H*G_tilde)'])



function V = skew(v)
V = zeros(3);
V(2,3) = -v(1);
V(1,3) = v(2);
V(1,2) = -v(3);
V(3,2) = v(1);
V(3,1) = -v(2);
V(2,1) = v(3);
end
