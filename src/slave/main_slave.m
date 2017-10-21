% test slave class
clc
clear
close all

% define some constants
SYN_ID = 2; % 1: grasping, 2: rotation

s = Slave();
for t = 0 : 10000
    s.move_synergy(SYN_ID, (1-cos(2*pi*0.001*t))/2)
end
