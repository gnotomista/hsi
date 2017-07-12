clc
close all
clear

hand = SGparadigmatic;
[qm, S] = SGsantelloSynergies;
hand = SGdefineSynergies(hand,S,qm);
hand = SGmoveHand(hand,qm);

%H = eye(4);
%H(1:3,4) = [0,70,-40];
%object = SGsphere(H,20,30);
% handActivatedJoints = ones(1,20)';
% figure(1)
% SGplotHand(hand);
% hold on
% SGplotSolid(object);
% grid on;
% axis('equal');
% title('Hand')
%[hand,object] = SGcloseHand(hand,object,handActivatedJoints,0.05);

hand = SGaddFtipContact(hand,1,1:3);
[hand,object] = SGmakeObject(hand); 

figure(1)
SGplotHand(hand);
hold on
SGplotObject(object);
grid on;
axis('equal');
title('SynergiesHand')

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

% evaluate the homogeneous quasi static solution
Gamma = SGquasistaticHsolution(hand,object);

% evaluate the kinematic manipulability ellipsoid
[kinmanipulability,ueig,zeig] = SGkinManipulability(Gamma);

% translate the kinematic ellipsoid on the pen tip
%
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
figure(1)
hold on
mesh(u1t,u2t,u3t)

ueig
zeig
