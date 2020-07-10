clc
clear
close all

% ros node initialization
rosshutdown
rosinit

topic_joints = '/sense_glove_joints';
topic_imu = '/sense_glove_imu';
topic_forces = '/sense_glove_forces';

sub_joints = rossubscriber(topic_joints,'sensor_msgs/JointState');
sub_imu = rossubscriber(topic_imu,'geometry_msgs/Pose');

pub_forces = rospublisher(topic_forces,'sensor_msgs/JointState');
msg_forces = rosmessage('sensor_msgs/JointState');

% init hand pose
msg_imu = receive(sub_imu, 1);
R0 = quat2rotm([msg_imu.Orientation.X msg_imu.Orientation.Y msg_imu.Orientation.Z msg_imu.Orientation.W]);

% collect data
N = 1e4;
sense_glove_data = zeros(N, 24); % 1 timestamp, 20 joints, 3 imu
for n = 1 : N
    disp([num2str(N-n), ' remaining data points to collect.'])
    
    [t_s, t_ns] = unixtime();
    t = t_s + t_ns;
    
    msg_joints = receive(sub_joints, 1);
    msg_imu = receive(sub_imu, 1);
    
    joint_angles = msg_joints.Position;
    pose_euler_angles = rotm2eul(quat2rotm([msg_imu.Orientation.X msg_imu.Orientation.Y msg_imu.Orientation.Z msg_imu.Orientation.W])*R0')';
    
    sense_glove_data(n,:) = [t, joint_angles', pose_euler_angles'];
end

dlmwrite(['sense_glove_data_', num2str(unixtime), '.csv'], ...
    sense_glove_data(1:n,:), ...
    'delimiter', ',', ...
    'precision', 18)


