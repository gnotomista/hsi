% test slave class
clc
clear
close all

% define some constants
N_ROBOTS = 6;
SYN_ID = 1;

% define network topology
L = diag(2*ones(N_ROBOTS,1));
for i = 1 : size(L,1)
    for j = 1 : size(L,2)
        if i ~= j  % i == mod(j-1,N_ROBOTS) || i == mod(j+1,N_ROBOTS) || j == mod(i-1,N_ROBOTS) || j == mod(i+1,N_ROBOTS)
            L(i,j) = -1;
        end
    end
end

s = Slave(N_ROBOTS, L);
s.move_synergy(SYN_ID, 0.5)
% for t = 0 : 10000
%     s.move_synergy(SYN_ID, (1-cos(2*pi*0.001*t))/2)
% end




