clc
clear
close all

% define some constants
LOAD = true;
N_ROBOTS = 10;
GRASP_ROTATE = 'grasp'; % or 'rotate'
SYNERGY_TO_VISUALIZE = 1;

% collect data & do PCA, or load pre-generated stuff
if LOAD
    if strcmp(GRASP_ROTATE, 'grasp')
        load('pc_grasping.mat')
    elseif strcmp(GRASP_ROTATE, 'rotate')
        load('pc_rotating.mat')
    end
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
    if strcmp(GRASP_ROTATE, 'grasp')
        v = grasp_polygon(N,L);
    elseif strcmp(GRASP_ROTATE, 'rotate')
        v = rotate_polygon(N,L);
    end
end

robotarium = initialize_robotarium(N,L);

visualize_synergy(robotarium,v,SYNERGY_TO_VISUALIZE);
