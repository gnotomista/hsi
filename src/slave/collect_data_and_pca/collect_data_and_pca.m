clc
clear
close all

% define some constants
N_ROBOTS = 6;
GRASP_ROTATE = 'rotate'; % 'grasp' or 'rotate'
SYNERGY_TO_VISUALIZE = 1;
OVERWRITE_MAT = true;
matfilenames = containers.Map({'grasp','rotate'}, ...
    {'../mat_files/synergies_grasp.mat','../mat_files/synergies_rotate.mat'});

% define network topology
Lapl = diag(2*ones(N_ROBOTS,1));
for i = 1 : size(Lapl,1)
    for j = 1 : size(Lapl,2)
        if i ~= j  % i == mod(j-1,N_ROBOTS) || i == mod(j+1,N_ROBOTS) || j == mod(i-1,N_ROBOTS) || j == mod(i+1,N_ROBOTS)
            Lapl(i,j) = -1;
        end
    end
end
% collect
if strcmp(GRASP_ROTATE, 'grasp')
    [synergies_vectors, synergies_mean] = grasp_polygon(N_ROBOTS,Lapl);%,'DT',0.01,'N_DATA',100,'PLOT_STUFF',false);
elseif strcmp(GRASP_ROTATE, 'rotate')
    [synergies_vectors, synergies_mean] = rotate_polygon(N_ROBOTS,Lapl,'PLOT_STUFF',false);
end

% delete old mat ...
if OVERWRITE_MAT
    delete(matfilenames(GRASP_ROTATE))
end
% ... and save new one
save(matfilenames(GRASP_ROTATE),'synergies_vectors','synergies_mean','Lapl')
