%% variables
t = 0:0.1:4;
[a,s] = arclenght(t(end),1,t');
s = -[s;1-s];

%% Master
qm = [-1.2826    -1.7596    0.4405   -0.0916  ...
       0.4533     0.3465    1.1698    0.8165  ...
       0          0.3454    1.2601    0.9067  ...
      -0.2229     0.2914    1.2981    0.8654  ...
      -0.5283     0.3394    1.1613    0.7675]';
hand = SGparadigmatic(T);
hand = SGmoveHand(hand,qm);
Syn = load('S.mat');                    
hand = SGdefineSynergies(hand,Syn.S,qm);

%% Slave

%% 
i = 1;
while (i<size(s,1)-1)
    hand = SGactivateSynergies(hand, [s(i+1)-s(i),0]');
    SGplotHand(hand)
    axis equal
    drawnow;
    i = i + 1;
end

