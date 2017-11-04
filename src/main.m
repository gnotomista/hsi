clc
clear
close all

%% variables
SYN_ID = 1;
t = 0:0.1:4;
[a,s] = arclength(t(end),1,t');
s = -[s;1-s];

%% Master
figure('units','normalized','position',[.05 .2 .45 .6])
T = eye(4);
qm = [-1.2826    -1.7596    0.4405   -0.0916  ...
       0.4533     0.3465    1.1698    0.8165  ...
       0          0.3454    1.2601    0.9067  ...
      -0.2229     0.2914    1.2981    0.8654  ...
      -0.5283     0.3394    1.1613    0.7675]';
hand = SGparadigmatic(T);
hand = SGmoveHand(hand,qm);
Syn = load('synergiesGraspObj.mat');                    
hand = SGdefineSynergies(hand,Syn.S,qm);

%% Slave
slave = Slave();
set(slave.robotarium_container.r.figure_handle,...
    'units','normalized','position',[.5 .2 .45 .6])

%% 
i = 1;
while (i<size(s,1)-1)
    figure(1)
    hand = SGactivateSynergies(hand, [s(i+1)-s(i),0]');
    SGplotHand(hand)
    axis equal
    drawnow;
    
    figure(2)
    slave.move_synergy(SYN_ID, s(i+1)+0.5)
    i = i + 1;
end

