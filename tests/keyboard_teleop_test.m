clc
clear
close all

%% Constants and init
SYN_ID = 1;
slave_frames = [];
t = 0:0.1:5;
[a,s] = arclength(t(end),1,t');
s = -2*s;

%% Slave
slave = Slave('mat_files/all_data');
slave.set_max_iter(4);
set(slave.robotarium_container.r.figure_handle,...
    'units','normalized','position',[.5 .2 .45 .6])
load_environment
obj = RigidBody([-.3 -0.3; -.3 0.3; 0.1 0.3; 0.3 -.2]);

%% Main
s = 0;
if (true)
    while (true)
        x = input("press some button", 's')
        switch x
            case "e"
                s = s + 0.1;
                v_obj_des = [0;0];
                omega_obj_des = 0;
            case "f"
                s = s - 0.1;
                v_obj_des = [0;0];
                omega_obj_des = 0;
            case "d"
                v_obj_des = [1;0];
                omega_obj_des = 0;
            case "a"
                v_obj_des = [-1;0];
                omega_obj_des = 0;
            case "w"
                v_obj_des = [0;1];
                omega_obj_des = 0;                
            case "s"
                v_obj_des = [0;-1];
                omega_obj_des = 0;                
            case "z"
                v_obj_des = [0;0];
                omega_obj_des = 1;
            case "x"    
                v_obj_des = [0;0];
                omega_obj_des = -1;                
            otherwise
                v_obj_des = [0;0];
                omega_obj_des = 0;
        end
        
        % position feedforward to slave
        slave.retrieve_poses();
        if isempty(slave.G)
            slave.swarm_syn(SYN_ID, s)
        else
            if size(null(slave.G),2) > 2*slave.N-3 ...
                    || ~inpolygon(0,0,slave.robot_poses(1,slave.robots_in_contact_ids),slave.robot_poses(2,slave.robots_in_contact_ids))
                slave.swarm_syn_and_opt_obj_manip(SYN_ID, s, zeros(2,1), 0)
            else
                slave.swarm_syn_and_opt_obj_manip(SYN_ID, s, v_obj_des, omega_obj_des)
            end
        end
        slave.step()
        
        % check and plot contact points
        [robot_idcs, contact_points, ~] = obj.check_contact(slave.robot_poses(1:2,:)', gcf);
        slave.update_grasp_matrix(robot_idcs, contact_points, obj.o_);
        
        obj.set_grasp_matrix(slave.G);           % set object grasp matrix
        obj.set_contact_points(contact_points);  % set object contact points
        
        % integrate object and restore robot positions
        contact_points_integrated_positions = obj.step(robot_idcs, slave.v, 5, 0); % this can easily simulate non-operational robots
        robot_integrated_positions = slave.robot_poses;
        robot_integrated_positions(1:2, robot_idcs) = contact_points_integrated_positions(:, robot_idcs);
        robot_integrated_positions = slave.check_robot_collisions(robot_integrated_positions);
        
        for i = 1 : slave.N
            % this needs some logic, e.g. if (contact changed || object pose changed)
            slave.set_theta_hinge(i, slave.robot_poses(3,i))
        end
        % slave.update_grasp_matrix(robot_idcs, contact_points, obj.o_(1:2,3));
        
        slave.overwrite_poses(robot_integrated_positions)
        % slave.plot_bb([0,1,0]);
        
        gcf;
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

