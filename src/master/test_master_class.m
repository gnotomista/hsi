clc
clear

m = Master('master/sense_glove/sense_glove_synergies.mat');

m.S
m.qm

m.get_joint_angles()
m.get_hand_pose()
m.get_hand_pose(true)
m.get_hand_synergies()

m.set_hand_forces([100,0,0,0,100,0,0,0,100,0,0,0,100,0,0,0,100,0,0,0])
pause(2)
m.set_hand_forces()
pause(2)
m.set_hand_synergistic_forces([16;-1])
pause(2)
m.set_hand_synergistic_forces()