clear
close all
clc

data = importdata('LeapData.txt');
coeff = pca(data);
qm = mean(data);

T = eye(4);
handopen = SGparadigmatic(T);
SGplotHand(handopen);
axis equal
handinit = SGmoveHand(handopen,[-1.2826    -1.7596    0.4405   -0.0916 ...
                                 0.4533     0.3465    1.1698    0.8165  ...
                                 0          0.3454    1.2601    0.9067  ...
                                -0.2229     0.2914    1.2981    0.8654  ...
                                -0.5283     0.3394    1.1613    0.7675]'); 
hand = SGmoveHand(handinit,qm);
figure
SGplotHand(hand)
axis equal
grid on;
title('Hand mean configuration');

hand = SGdefineSynergies(hand,coeff(:,1),qm); 
hand = SGactivateSynergies(hand, 0.1);
figure
SGplotHand(hand)
axis equal
grid on;
title('Hand activated synergies');