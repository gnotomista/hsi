clear
close all
clc

figure(1)
T = eye(4);
handopen = SGparadigmatic(T);
SGplotHand(handopen);
axis equal

% [qm, S] = SGsantelloSynergies;
% handopen = SGdefineSynergies(handopen,S,1:15);
handinit = SGmoveHand(handopen,[-1.2826    -1.7596    0.4405   -0.0916 ...
                                 0.4533     0.3465    1.1698    0.8165  ...
                                 0          0.3454    1.2601    0.9067  ...
                                -0.2229     0.2914    1.2981    0.8654  ...
                                -0.5283     0.3394    1.1613    0.7675]'); 
                               
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

% rotAngleObj = -25*pi/180;
% R = SGrotz(rotAngleObj);

i=0;
range = [0 0.05];
data = [];
obj.base = [eye(3) obj.center; zeros(1,3) 1];

while i<100
    hand = handinit;
    initftips = hand.ftips+(range(1) + (range(2)-range(1))*rand(3,5));
    [e,hand] = inverseKinematics(hand,initftips);
    data = [data,hand.q];
    r = initftips - obj.center;
    for j = 1:25
        rotAngleObj = -j*pi/180;
        R = SGrotz(rotAngleObj);
        desFtips = R*r + obj.center;
        [e,hand] = inverseKinematics(hand,desFtips);
        data = [data,hand.q];
    end
    i=i+1
end

figure(3)
H(1:3,1:3) = R;
H(1:3,4) = [0,70,-50];
obj = SGcube(H,70,50,22);
SGplotHand(hand);
axis equal
SGplotSolid(obj);
grid on;
title('Hand final configuration')

coeff = pca(data');
qm = mean(data');
hand = SGmoveHand(handinit,qm);
figure
SGplotHand(hand)
axis equal
grid on;
title('Hand mean configuration');

hand = SGdefineSynergies(hand,coeff(:,1),qm); 
hand = SGactivateSynergies(hand, 0.1);
figure
SGplotHand(hand)
axis equal
grid on;
title('Hand activated synergies');