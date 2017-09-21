clc
clear
close all

% define some constants
LOAD = true;
N_ROBOTS = 10;
SYNERGY_TO_VISUALIZE = 2;

% collect data & PCA, or load pre-generated stuff
if LOAD
    load('pc.mat')
    N = size(L,1);
else
    N = N_ROBOTS;
    % define network topology
    L = diag(2*ones(N,1));
    for i = 1 : size(L,1)
        for j = 1 : size(L,2)
            if i ~= j  % i == mod(j-1,N_ROBOTS) || i == mod(j+1,N_ROBOTS) || j == mod(i-1,N_ROBOTS) || j == mod(i+1,N_ROBOTS)
                L(i,j) = -1;
            end
        end
    end
    % collect 
    v = grasp_polygon(N,L);
end

robotarium = initialize_robotarium(N,L);

visualize_synergy(robotarium,v,SYNERGY_TO_VISUALIZE);
