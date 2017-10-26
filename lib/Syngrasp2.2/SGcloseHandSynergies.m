function [newHand,object] = SGcloseHandSynergies(hand_,obj,activeSynergies_,increment)

final_pose = 0;

if (isscalar(increment))
    step = increment * ones(size(hand_.S,2),1);
else
    step = increment;
end
activeSynergies = activeSynergies_;
activeJoints = hand_.S*activeSynergies;
activeJoints = (abs(activeJoints)>0);

count = 1;
max_iter = 1000;
hand{1} = hand_;

z = pinv(hand{1}.S)*(hand{1}.q - hand_.offset);

while (final_pose == 0 && count <= max_iter)


    q_new = hand{count}.q + hand{1}.S*(activeSynergies.*increment)
    for k = 1:length(hand{count}.q)
        if(activeJoints(k) ~= 0 && q_new(k) >= hand{count}.limit(k,1) && q_new(k) <= hand{count}.limit(k,2))
        else
            activeJoints(k) = 0;
        end
    end
    
    z = pinv(hand{1}.S)*(q_new - hand{count}.q);
    activeSynergies = pinv(hand{1}.S)*activeJoints;
    activeSynergies = (abs(activeSynergies)>0).*activeSynergies_;
    
    hand{count} = SGactivateSynergies(hand{count},activeSynergies.*z);

    %debug
    figure(8);
    SGplotHand(hand{count});
    grid on;
    axis equal;
    view(90, 0);
    hold on
    SGplotSolid(obj);
    pause;

    for i = 1 : size(hand{1}.F,2)
        index = find(hand{count}.qin == i);
        [cp_mat] = SGcontactDetection(hand{count},obj,i);
        if ~isempty(cp_mat)
            max_link = max(cp_mat(:,1));
            for h = 1:max_link
                activeJoints(index(h)) = 0;
            end
            for c=1:size(cp_mat,1)
                hand{count} = SGaddContact(hand{count},1,i,cp_mat(c,1),cp_mat(c,2));
            end
        end
    end
    activeSynergies = pinv(hand{1}.S)*activeJoints;
    activeSynergies = (activeSynergies>0.999).*activeSynergies_;
    
    if (activeSynergies == zeros(size(hand{1}.S,2),1))
        final_pose = 1;
    end
    %count
    %     for i=1:hand{count}.n % for each finger
    %         index = find(hand{count}.qin == i);
    %         for j = 1:length(index) % for each joint
    %             k = index(j);
    %             if(activeJoints(k) == 1)
    %                 q_new = hand{count}.q;
    %                 q_new(k) = q_new(k) + step(k);
    %                 if(q_new(k) >= hand{count}.limit(k,1) && q_new(k) <= hand{count}.limit(k,2))
    %                     hand{count} = SGmoveHand(hand{count},q_new);
    %                     % contact detection
    %                     [cp_mat] = SGcontactDetection(hand{count},obj,i);
    %                     if ~isempty(cp_mat)
    %                         max_link = max(cp_mat(:,1));
    %                         for h = 1:max_link
    %                             activeJoints(index(h)) = 0;
    %                         end
    %                         for c=1:size(cp_mat,1)
    %                             hand{count} = SGaddContact(hand{count},1,i,cp_mat(c,1),cp_mat(c,2));
    %                         end
    %                     end
    %                 else
    %                     activeJoints(k) = 0;
    %                 end
    %             end
    %         end
    %     end
    %
    %     if (activeJoints == zeros(length(hand{count}.q),1))
    %         final_pose = 1;
    %     end
    hand{count+1} = hand{count};
    count = count +1;
    activeSynergies
    
end

% for i = 1:count
%     fig = figure(i);
%     SGplotHand(hand{i});
%     axis('equal');
%     hold on
%     SGplotSolid(obj);
% end
if size(hand{count}.cp,2)>0
    [hand{count},object] = SGcontact(hand{count},obj);
else
    object = obj;
    object.base = [eye(3) object.center; zeros(1,3) 1];
    object.Kc = [];
    object.H = [];
    object.Gtilde = [];
    object.G = [];
    disp('Contact not found')
end
newHand = hand{count};
end