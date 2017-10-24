clear
close all
clc

d = importdata('LeapData.txt');
d = d.data(:,2:end);
coeff = pca(d);
qm = mean(d);

T = eye(4);
hand = SGparadigmatic(T);
hand = SGmoveHand(hand,qm);
figure
SGplotHand(hand)
axis equal
grid on;
title('Hand mean configuration');

hand = SGdefineSynergies(hand,coeff(:,1:2),qm); 
hand = SGactivateSynergies(hand, [0, 0]');
figure
SGplotHand(hand)
axis equal
grid on;
title('Hand activated synergies');