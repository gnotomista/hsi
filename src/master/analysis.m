clear
clc

load('synergiesGraspRot');

qm(1) = qm(1)+0.5;
qm(2) = qm(2)+0.7;

hand = SGparadigmatic;
hand = SGdefineSynergies(hand,S(:,1:2),qm); 
hand.offset = qm;

posei = hand.ftips;
posef = [  -15  -27   -3   13    31
            65  114  100  114   114
           -44  -46  -52  -50  -46];

hand = SGmoveHand(hand,qm);

[e,hand] = inverse_kinematics_synergies(hand,posef);

%H = eye(4);
%H(1:3,4) = [0,90,-50];
%obj = SGcube(H,65,50,30);

hand = SGaddFtipContact(hand,1,1:5);
[hand,object] = SGmakeObject(hand); 

figure
SGplotHand(hand);
hold on
SGplotObject(object);
grid on;
axis('equal');
title('HandInitConfig');

delta_zr = [1 0]';
variation = SGquasistatic(hand,object,delta_zr);
linMap = SGquasistaticMaps(hand,object);
rbmotion = SGrbMotions(hand,object);

[Gamma] = SGquasistaticHsolution(hand, object);
kinmanipulability = SGkinManipulability(Gamma);
forcemanipulability = SGforceManipulability(Gamma);

% [hand,obj] = SGcloseHandSynergies(hand,obj,[1,0]',0.11);