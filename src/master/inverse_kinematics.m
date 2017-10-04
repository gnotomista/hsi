function [e,hand] = inverse_kinematics(hand,pose)

deltaT = 0.001;
hand_ = hand;
K = 0.2*eye(3*size(hand_.ftips,2));
error = reshape(pose,3*size(pose,2),1) - reshape(hand_.ftips,3*size(hand_.ftips,2),1);
i = 1;

% figure(4)
% SGplotHand(hand_);
% hold on
% grid on;
% axis('equal');
% title('InverseKinematics')

hand_ = SGaddFtipContact(hand_,1,1:size(hand_.ftips,2));
[hand_,obj] = SGmakeObject(hand_);

while (norm(error) > 0.01 && i < 500)
    hand_.Jtilde = SGjacobianMatrix(hand_);
    H = SGselectionMatrix(obj);
    obj.H = H;
    hand_.H = H;
    hand_.J = H*hand_.Jtilde;
    %J = [hand_.J(7:9,:);hand_.J(1:6,:)];
    %hand_ = SGactivateSynergies(hand_,deltaT*pinv(hand_.S)*J'*K*error);
    
    hand_ = SGmoveHand(hand_,hand_.q + deltaT*hand_.J'*K*error);
    error = reshape(pose,3*size(pose,2),1) - reshape(hand_.ftips,3*size(hand_.ftips,2),1);
    e(i,:) = error;
    i = i+1;
%     hand_.J;
%     figure(8);
%     SGplotHand(hand_);
%     grid on;
%     axis equal;
%     pause;

end

hand_.cp = [];
hand_.J = [];
hand = hand_;

end