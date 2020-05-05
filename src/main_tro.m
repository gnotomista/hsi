clc
clear
close all

%% Constants and init
SIMULATED_MASTER = 1;
SYN_ID = 1;

master_frames = [];
slave_frames = [];

if ~SIMULATED_MASTER
    rosshutdown
end

%% Master
Syn = load('synergiesGraspRot.mat');
if (SIMULATED_MASTER)
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
    drawnow
else
    rosinit
    jointsub = rossubscriber('/leap_motion_joints','sensor_msgs/JointState');
    message = receive(jointsub,10);
end

%% Slave
slave = Slave('mat_files/all_data');
slave.set_max_iter(1);
set(slave.robotarium_container.r.figure_handle,...
    'units','normalized','position',[.5 .2 .45 .6])
figure(2)

obj = RigidBody([-.3 -0.3; -.3 0.3; 0.1 0.3; 0.3 -.2]);

%% Main
if (SIMULATED_MASTER)
    for t = 1 : size(s)-1
        
        % update master
        figure(1)
        hand = SGactivateSynergies(hand, -[s(t+1)-s(t),0]');
        % [v_des, omega_des] = get_v_omega_des();
        v_obj_des = [0;1]; % setting these to something nonzero when few robots are in contact can easily lead to infeasibility of the opt probl
        omega_obj_des = 0;
        % plot master
        SGplotHand(hand)
        axis equal
        axis manual
        view(-130,25)
        axis off
        drawnow
        master_frames = [master_frames, getframe(gcf)];
        
        % position feedforward to slave
        slave.retrieve_poses();
        if isempty(slave.G)
            slave.swarm_syn(SYN_ID, s(t))
        else
            if size(null(slave.G),2) > 2*slave.N-3 ...
                    || ~inpolygon(0,0,slave.robot_poses(1,slave.robots_in_contact_ids),slave.robot_poses(2,slave.robots_in_contact_ids))
                slave.swarm_syn_and_opt_obj_manip(SYN_ID, s(t), zeros(2,1), 0)
            else
                slave.swarm_syn_and_opt_obj_manip(SYN_ID, s(t), v_obj_des, omega_obj_des)
            end
        end
        slave.step()
        
        % check and plot contact points
        [robot_idcs, contact_points, ~] = obj.check_contact(slave.robot_poses(1:2,:)', figure(2));
        slave.update_grasp_matrix(robot_idcs, contact_points, obj.o_);
        
        obj.set_grasp_matrix(slave.G);           % set object grasp matrix
        obj.set_contact_points(contact_points);  % set object contact points
        
        % integrate object and restore robot positions
        contact_points_integrated_positions = obj.step(robot_idcs, slave.v, 0, 0); % this can easily simulate non-operational robots
        robot_integrated_positions = slave.robot_poses;
        robot_integrated_positions(1:2, robot_idcs) = contact_points_integrated_positions(:, robot_idcs);
        
        for i = 1 : slave.N
            % this needs some logic, e.g. if (contact changed || object pose changed)
            slave.set_theta_hinge(i, slave.robot_poses(3,i))
        end
        % slave.update_grasp_matrix(robot_idcs, contact_points, obj.o_(1:2,3));
        
        slave.overwrite_poses(robot_integrated_positions)
        
        figure(2)
        slave_frames = [slave_frames, getframe(gcf)];
        % force feedback to master
        % ---------------------------------------------------------- ACTUAL
        % master.set_forces(forces) % or show_forces(forces)
        % ------------------------------------------------------------ TEST
        % ... do nothing ...
        % ------------------------------------------------------------- END
    end
else % if (SIMULATED_MASTER)
    % while (1)
    %     % read master joints
    %     q = receive(jointsub,10);
    %
    %     % calculate synergies
    %     z = pinv(Syn.S(:,1:2))*q.Position
    %
    %     slave.move_synergy(1, z(1))
    % end
end

