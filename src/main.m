%% variables
t = 0:0.1:4;
[a,s] = arclenght(t(end),1,t');
s = -[s;1-s];

%% Master
figure(1)
T = eye(4);
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
N_ROBOTS = 6;
SYN_ID = 1;
L = diag(2*ones(N_ROBOTS,1));
for i = 1 : size(L,1)
    for j = 1 : size(L,2)
        if i ~= j  % i == mod(j-1,N_ROBOTS) || i == mod(j+1,N_ROBOTS) || j == mod(i-1,N_ROBOTS) || j == mod(i+1,N_ROBOTS)
            L(i,j) = -1;
        end
    end
end
slave = Slave(N_ROBOTS, L);

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

