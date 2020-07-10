clc
clear
close all

load sense_glove_synergies.mat

rosshutdown
rosinit

topic_joints = '/sense_glove_joints';

sub_joints = rossubscriber(topic_joints,'sensor_msgs/JointState');

figure, hold on, set(gca,'visible','off'), ylim([-5 5])
hb = bar([0 0]);

while true
    msg_joints = receive(sub_joints, 1);
    joint_angles = msg_joints.Position;
    
    synergies_components = S'*(joint_angles-qm');
    
    hb.YData = synergies_components;
end


