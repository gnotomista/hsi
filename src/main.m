clear
close all
clc

figure(1)
T = eye(4);
handopen = SGparadigmatic(T);
SGplotHand(handopen);
axis equal

[qm, S] = SGsantelloSynergies;
handopen = SGdefineSynergies(handopen,S,1:15);
handinit = SGmoveHand(handopen,[-1.2826    -1.7596    0.4405   -0.0916 ...
                                   0.4533     0.3465    1.1698    0.8165  ...
                                   0          0.3454    1.2601    0.9067  ...
                                   -0.2229    0.2914    1.2981    0.8654  ...
                                   -0.5283    0.3394    1.1613    0.7675]');                    
                               
figure(2)
SGplotHand(handinit);
axis equal

H = eye(4);
H(1:3,4) = [0,70,-50];
obj = SGcube(H,70,50,22);

SGplotHand(handinit);
axis equal
SGplotSolid(obj);
grid on;
title('Hand init configuration')

rotAngleObj = -25*pi/180;
R = SGrotz(rotAngleObj);
r = handinit.ftips - obj.center;

desFtips = R*r + obj.center;
obj.base = [eye(3) obj.center; zeros(1,3) 1];
handinit = SGaddFtipContact(handinit,1,1:5);
[handinit,obj] = SGmakeObject(handinit);
[e,hand] = inverse_kinematics(handinit,desFtips,obj);

figure(3)
H(1:3,1:3) = R;
H(1:3,4) = [0,70,-50];
obj = SGcube(H,70,50,22);
SGplotHand(hand);
axis equal
SGplotSolid(obj);
grid on;
title('Hand final configuration')