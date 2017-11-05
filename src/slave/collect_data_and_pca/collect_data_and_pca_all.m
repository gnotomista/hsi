clc
clear
close all

% load topology
load('../mat_files/graph_topology')

% collect
[~,~,data_for_pca1,Lapl] = grasp_polygon('../mat_files/graph_topology');%,'DT',0.01,'N_DATA',100,'PLOT_STUFF',false);
[~,~,data_for_pca2] = rotate_polygon('../mat_files/graph_topology','PLOT_STUFF',false);
data_for_pca = [data_for_pca1, data_for_pca2];

% pca
[synergies_vectors,~,~,~,~,synergies_mean] = pca(data_for_pca');

save('../mat_files/synergies_all','synergies_vectors','synergies_mean','Lapl')
