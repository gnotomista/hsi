classdef Master
    properties
        S
        qm
    end
    properties (Access=private)
        topic_joints
        topic_imu
        topic_forces
        sub_joints
        sub_imu
        pub_forces
        msg_forces
        R0
    end
    
    methods
        function this = Master(synergy_mat_file_name)
            load(synergy_mat_file_name)
            this.S = S;
            this.qm = qm;
            % init ros
            rosshutdown
            rosinit
            % topics of interest
            this.topic_joints = '/sense_glove_joints';
            this.topic_imu = '/sense_glove_imu';
            this.topic_forces = '/sense_glove_forces';
            % subscribers
            this.sub_joints = rossubscriber(this.topic_joints,'sensor_msgs/JointState');
            this.sub_imu = rossubscriber(this.topic_imu,'geometry_msgs/Pose');
            % publishers
            this.pub_forces = rospublisher(this.topic_forces,'sensor_msgs/JointState');
            this.msg_forces = rosmessage('sensor_msgs/JointState');
            % init hand pose
            msg_imu = receive(this.sub_imu, 1);
            this.R0 = quat2rotm([msg_imu.Orientation.X msg_imu.Orientation.Y msg_imu.Orientation.Z msg_imu.Orientation.W]);
        end
        
        function joint_angles = get_joint_angles(this)
            msg_joints = receive(this.sub_joints, 1);
            joint_angles = msg_joints.Position;
        end
        
        function hand_orientation = get_hand_pose(this, round_degree)
            msg_imu = receive(this.sub_imu, 1);
            hand_orientation = rotm2eul(quat2rotm([msg_imu.Orientation.X msg_imu.Orientation.Y msg_imu.Orientation.Z msg_imu.Orientation.W])*this.R0')';
            if nargin > 1
                if round_degree
                    hand_orientation = round(180/pi*hand_orientation);
                end
            end
        end
        
        function hand_synergies = get_hand_synergies(this)
            joint_angles = this.get_joint_angles();
            hand_synergies = this.S' * joint_angles;
        end
        
        function set_hand_forces(this, joint_forces)
            % forces : 20x1 torques on each joint
            if nargin > 1                
                finger_forces = this.calculate_finger_forces(joint_forces); % 5x1 torques on each finger
                this.msg_forces.Effort = finger_forces;
            else
                this.msg_forces.Effort = [0,0,0,0,0];
            end
            this.pub_forces.send(this.msg_forces);
        end
        
        function set_hand_synergistic_forces(this, syn_forces)
            % syn_forces : 2x1 torques on each synergy
            if nargin > 1
                joint_forces = this.S * syn_forces;
                finger_forces = this.calculate_finger_forces(joint_forces);
                this.msg_forces.Effort = finger_forces;
            else
                this.msg_forces.Effort = [0,0,0,0,0];
            end
            this.pub_forces.send(this.msg_forces);
        end
    end
    methods (Access=private)
        function finger_forces = calculate_finger_forces(this, joint_forces)
            % map torques from each finger joint to the first joint (using partial jacobian transposes) and sum them up
            % dummy mapping, i.e. only take the torques on the first joints, without summing the contributions of the other joints of each fingers
            finger_forces = joint_forces([1,5,9,13,17]);
            finger_forces = max(0, min(100, finger_forces));
        end
    end
end
