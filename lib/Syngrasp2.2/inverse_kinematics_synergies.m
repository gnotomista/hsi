function [e,hand] = inverse_kinematics_synergies(hand,pose)

deltaT = 0.001;
hand_ = hand;
K = 0.08*eye(3*size(hand_.ftips,2));
Kf = 0.08*eye(3*size(hand_.ftips,2));
error = reshape(pose,3*size(pose,2),1) - reshape(hand_.ftips,3*size(hand_.ftips,2),1);
i = 1;
e(1,:) = error;

% x = reshape(hand_.ftips,3*size(hand_.ftips,2),1);
% L = [2,-1,-1;
%     -1,2,-1;
%     -1,-1,2];

% gamma = eye(3);
% F = kron(-L,gamma)*x;

hand_.cp = [];
hand_.J = [];

hand_ = SGaddFtipContact(hand_,1,1:size(hand_.ftips,2));
[hand_,obj] = SGmakeObject(hand_);

while (norm(error) > 0.01 && i < 1000)
    hand_.Jtilde = SGjacobianMatrix(hand_);
    H = SGselectionMatrix(obj);
    obj.H = H;
    hand_.H = H;
    hand_.J = H*hand_.Jtilde;
    hand_ = SGactivateSynergies(hand_,deltaT*pinv(hand_.S)*hand_.J'*(K*error));
    x = reshape(hand_.ftips,3*size(hand_.ftips,2),1);
%     F = kron(-L,gamma)*x;
    error = reshape(pose,3*size(pose,2),1) - reshape(hand_.ftips,3*size(hand_.ftips,2),1);
    e(i,:) = error;
    i = i+1;
end

hand_.cp = [];
hand_.J = [];

hand = hand_;

end