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

hand = SGaddFtipContact(hand,1,1:3);
[hand,object] = SGmakeObject(hand); 

figure
SGplotHand(hand);
hold on
SGplotObject(object);
grid on;
axis('equal');
title('HandInitConfig');

hand.Jtilde = SGjacobianMatrix(hand);
H = SGselectionMatrix(object);
object.H = H;
hand.H = H;
hand.J = H*hand.Jtilde;
object.Gtilde = SGgraspMatrix(object);
object.G = object.Gtilde*hand.H';

[nl,nq]= size(hand.J);
[nd]= size(object.G,1);
% 
% choose the synergy indexes
 syn_index = 1:6;
% 
% 
% choose the corresponding columns
 S_rid = S(:,syn_index);
 hand.S = S_rid;
 nz = size(hand.S,2);

 
% define the stiffness matrices
% 
 Ks = eye(nl);
 Kq = eye(nq);
 Kz = eye(nz);
 
object = SGcontactStiffness(object,Ks);
hand = SGjointStiffness(hand,Kq);
hand = SGsynergyStiffness(hand,Kz);
  
% 
 %%%%% constant synergy matrix
 Ksz = zeros(nz,nz);
 Kjq = zeros(nq,nq);
 Kju = zeros(nq,nd);


delta_zr = [1,0,0,0,0,0]';
variation = SGquasistatic(hand,object,delta_zr);
linMap = SGquasistaticMaps(hand,object);
rbmotion = SGrbMotions(hand,object);

[Gamma] = SGquasistaticHsolution(hand, object);
kinmanipulability = SGkinManipulability(Gamma);

kinellips = kinmanipulability.kinellips;
[r,c] = size(kinellips.u1);
for i = 1:r
    for j = 1:c

        u1t(i,j) = kinellips.u1(i,j)+object.center(1);
        u2t(i,j) = kinellips.u2(i,j)+object.center(2);
        u3t(i,j) = kinellips.u3(i,j)+object.center(3);
    end
end
% 
% draw the kinematic manipulability ellipsoid in the workspace
% 
 hold on
 axis([-60 30 40 120 -100 50])
 mesh(u1t,u2t,u3t)

% [hand,obj] = SGcloseHandSynergies(hand,obj,[1,0]',0.11);