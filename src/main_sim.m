clc
clear
close all
rosshutdown

%% variables
SYN_ID = 1;
SIMULATED_MASTER = 0;

if (SIMULATED_MASTER)
    t = 0:0.1:4;
    [a,s] = arclength(t(end),1,t');
    s = -[s;1-s];
end

%% Master
Syn = load('synergiesGraspObj.mat');
if (SIMULATED_MASTER)
    figure('units','normalized','position',[.05 .2 .45 .6])
    T = eye(4);
    qm = [-1.2826    -1.7596    0.4405   -0.0916  ...
        0.4533     0.3465    1.1698    0.8165  ...
        0          0.3454    1.2601    0.9067  ...
        -0.2229     0.2914    1.2981    0.8654  ...
        -0.5283     0.3394    1.1613    0.7675]';
    hand = SGparadigmatic(T);
    hand = SGmoveHand(hand,qm);
    
    hand = SGdefineSynergies(hand,Syn.S,qm);
else
    rosinit
    jointsub = rossubscriber('/leap_motion_joints','sensor_msgs/JointState');
    message = receive(jointsub,10);
end

%% Slave
slave = Slave('slave/mat_files/all_data');
set(slave.robotarium_container.r.figure_handle,...
    'units','normalized','position',[.5 .2 .45 .6])

%%
if (SIMULATED_MASTER)
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
else
    while (1)        
        figure(1)
        
        % read master joints
        q = receive(jointsub,10);
        
        % calculate synergies
        z = pinv(Syn.S)*q.Position
        SYN_ID = max(z);
        
        
        %slave.move_synergy(SYN_ID, s(i+1)+0.5)
        i = i + 1;
    end
end

