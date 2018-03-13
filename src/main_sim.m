clc
clear
close all
rosshutdown

%% variables
SYN_ID = 2;
SIMULATED_MASTER = 1;

if (SIMULATED_MASTER)
    t = 0:0.1:4;
    [a,s] = arclength(t(end),1,t');
    s = -[s;1-s];
end

%% Master
Syn = load('synergiesGraspRot.mat');
if (SIMULATED_MASTER)
    figure('units','normalized','position',[.05 .2 .45 .6])
    T = eye(4);
    Syn.qm(1) = Syn.qm(1)+0.787;
    qm = Syn.qm;%[-1.2826    -1.7596    0.4405   -0.0916  ...
           %0.4533     0.3465    1.1698    0.8165  ...
           %0          0.3454    1.2601    0.9067  ...
           %-0.2229     0.2914    1.2981    0.8654  ...
           %-0.5283     0.3394    1.1613    0.7675]';
    hand = SGparadigmatic(T);
    hand = SGmoveHand(hand,qm);
    
    hand = SGdefineSynergies(hand,Syn.S(:,1:2),qm);
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
    j = 1;
    t = 1:10:size(s,1);
    while (i<size(s,1)-1)
        figure(1)
        set(gcf,'color','w');
        
        hand = SGactivateSynergies(hand, -[0,s(i+1)-s(i)]');
        SGplotHand(hand)
        axis equal
        axis manual
        view(-130,25)
        axis off

        drawnow;
        
        figure(2)
        slave.move_synergy(SYN_ID, s(i+1)+0.5)
        if i == t(j)
           j = j+1;
           figure(1)
           export_fig(strcat(strcat('master',int2str(j)),'.png'))  
           figure(2)
           export_fig(strcat(strcat('slave',int2str(j)),'.png'))  
        end
        i = i + 1;
        
    end
else
    while (1)        
        figure(1)
        
        % read master joints
        q = receive(jointsub,10);
        
        % calculate synergies
        z = pinv(Syn.S(:,1:2))*q.Position
        %SYN_ID = max(z);
        
        %slave.move_synergy(1, z(1))
        i = i + 1;
    end
end

