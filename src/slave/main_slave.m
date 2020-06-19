% test slave class
clc
clear
close all

%% Master
SYN_ID = 1;
t = 0:0.1:4;
[a,s] = arclength(t(end),1,t');
s = -2*s;
T = eye(4);
figure('units','normalized','position',[.05 .2 .45 .6])
Syn = load('synergiesGraspRot.mat');
Syn.qm(1) = Syn.qm(1)+0.787;
qm = Syn.qm;
hand = SGparadigmatic(T);
hand = SGmoveHand(hand,qm);
hand = SGdefineSynergies(hand,Syn.S(:,1:2),qm);
hand = SGactivateSynergies(hand, -[s(2)-s(1),0]');
SGplotHand(hand)
set(gcf,'color','w');
axis equal
axis manual
view(-130,25)
axis off
drawnow;

%% Slave
slave = Slave('mat_files/all_data');
slave.set_max_iter(6);
set(slave.robotarium_container.r.figure_handle,...
    'units','normalized','position',[.5 .2 .45 .6])
obj = RigidBody([-.3 -0.3; -.3 0.3; 0.1 0.3; 0.3 -.2]);

master_frames = [];
slave_frames = [];

for t = 1 : size(s)-1
    
    %% update master
    figure(1)
    hand = SGactivateSynergies(hand, -[s(t+1)-s(t),0]');
    SGplotHand(hand)
    axis equal
    axis manual
    view(-130,25)
    axis off
    drawnow;
    master_frames = [master_frames, getframe(gcf)];
    
    %% update slave
    figure(2)
    slave.swarm_syn(SYN_ID, s(t));
    slave.step()
    
    % compute grasp matrix
    [robot, minDist, edge, u, v] = obj.check_collision(slave.robot_poses(1:2,:)');   % check collisions
    if size(robot,2)~= size(slave.robots_in_contact,1)                               % if a new collision has occurred
        coll_robots = setdiff(robot, slave.robots_in_contact);                       % retrieve the new colliding robot(s)
        for i = 1 : size(coll_robots,2)                                              % for all the new colliding robots
            index = find(robot==coll_robots(i));                                     % find the index in the vector of colliding robots
            rp = slave.robot_poses(1:2, coll_robots(i));                             % retrieve its position
            c = edge(index,1:2)' + u(index)*(edge(index,3:4)-edge(index,1:2))';      % reconstruct the contact point (nearest point on the object border)
            oc = c - obj.o_(1:2,3);
            goc = eye(3); goc(1:2,3) = oc;
            slave.G(:, 2*coll_robots(i)-1:2*coll_robots(i)) = gi(inv(goc));
            % plot
            plot(c(1),c(2), 'r*')                                                    % plot contact point on the object
            drawnow
            slave.robots_in_contact = robot;
        end
    end
    slave_frames = [slave_frames, getframe(gcf)];
    
end

slave.G
