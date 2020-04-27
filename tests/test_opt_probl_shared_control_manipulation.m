clc
clear
close all

load('opt_probl_setup.mat')

slave.set_theta_max(pi/2);

v_obj_des = [0;0];
omega_obj_des = pi/4;

slave.shared_obj_manipulation(v_obj_des, omega_obj_des);