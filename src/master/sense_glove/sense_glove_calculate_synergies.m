clc
clear
close all

% synergy 1
d1 = csvread('sense_glove_data_syn1_1594331442.csv');
d1 = d1(1:end-1, 2:21);
% synergy 2
d2 = csvread('sense_glove_data_syn2_1594331758.csv');
d2 = d2(1:end-1, 2:21);
% all data
d = [d1; d2];

% pca
[synergies, ~, var, ~, ~, qm] = pca(d);

S = synergies(:,1:2);

plot((d-qm)*S)

save('sense_glove_synergies','S','qm')

% % synergy 1
% d1 = csvread('sense_glove_data_syn1_1594331442.csv');
% d1 = d1(1:end-1, 2:21);
% % pca
% s1 = pca(d1);
% % synergy 2
% d2 = csvread('sense_glove_data_syn2_1594331758.csv');
% d2 = d2(1:end-1, 2:21);
% % pca
% s2 = pca(d2);
% 
% S = [s1(:,1) s2(:,1)];
% 
% plot([d1-mean(d1);d2-mean(d2)]*S)

% % synergy 1 & 2
% d2 = csvread('sense_glove_data_syn2_1594331758.csv');
% d2 = d2(1:end-1, 2:21);
% 
% % pca
% [synergies, ~, var, ~, ~, qm] = pca(d2);
% 
% S = synergies(:,1:2);
% 
% d1 = csvread('sense_glove_data_syn1_1594331442.csv');
% d1 = d1(1:end-1, 2:21);
% plot(([d1;d2]-qm)*S)
