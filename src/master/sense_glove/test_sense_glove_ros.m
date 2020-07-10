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

% test
for n = 1 : 10
    msg_joints = receive(sub_joints, 1);
    msg_imu = receive(sub_imu, 1);
    joint_angles = msg_joints.Position;
    pose_euler_angles = round(180/pi*rotm2eul(quat2rotm([msg_imu.Orientation.X msg_imu.Orientation.Y msg_imu.Orientation.Z msg_imu.Orientation.W])*R0'))';
    disp('Joint angles:')
    disp(joint_angles')
    disp('IMU angles:')
    disp(pose_euler_angles')
    disp('---')
end

