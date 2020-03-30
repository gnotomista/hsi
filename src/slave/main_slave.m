% test slave class
clc
clear
close all

% define some constants
SYN_ID = 1; % 1: grasping, 2: rotation

s = Slave('mat_files/all_data');
o = RigidBody([-.6 -0.6; -.6 0.6; 0.2 0.6; 0.6 -.4]);
for t = 0 : 100
    s.move_synergy(SYN_ID, (1-cos(2*pi*0.1*t))/2)
    [robot, minDist, edge, u, v] = o.check_collision(s.robot_poses(1:2,:)');
    for i = 1 : size(robot,2)
       rp = s.robot_poses(1:2, robot(i));
       minDistPt = edge(i,1:2) + u(i)*(edge(i,3:4)-edge(i,1:2));
       h = line([rp(1),minDistPt(1)],[rp(2),minDistPt(2)]);
       drawnow
       delete(h)
    end
end
