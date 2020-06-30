
clc
clear
close all

%% Constants and init
SYN_ID = 1;
slave_frames = [];
t = 0:0.1:5;
[a,s] = arclength(t(end),1,t');
s = -2*s;
s_max = 2;
s_min = -2;
v_obj_des_max = 20;
omega_obj_des_max = 20;

%% Slave
slave = Slave('mat_files/all_data');
slave.set_max_iter(1);
set(slave.robotarium_container.r.figure_handle,...
    'units','normalized','position',[0 0 1 1])
load_environment
obj = RigidBody([-.3 -0.3; -.3 0.3; 0.1 0.3; 0.3 -.2]);
axis([-2 7 -2.5 5])
set(gcf,'KeyPressFcn',@teleop);

%% Main
global keyPressed
keyPressed = '';
s = 0;
contact_points = [];
if (true)
    while (true)
        switch keyPressed
            case "e"
                s = min(s_max,s+0.05);
                v_obj_des = [0;0];
                omega_obj_des = 0;
                keyPressed = '';
            case "f"
                s = max(s_min,s-0.05);
                v_obj_des = [0;0];
                omega_obj_des = 0;
                keyPressed = '';
            case "d"
                v_obj_des = v_obj_des_max*[1;0];
                omega_obj_des = 0;
                keyPressed = '';
            case "a"
                v_obj_des = v_obj_des_max*[-1;0];
                omega_obj_des = 0;
                keyPressed = '';
            case "w"
                v_obj_des = v_obj_des_max*[0;1];
                omega_obj_des = 0;
                keyPressed = '';
            case "s"
                v_obj_des = v_obj_des_max*[0;-1];
                omega_obj_des = 0;
                keyPressed = '';
            case "z"
                v_obj_des = [0;0];
                omega_obj_des = omega_obj_des_max*1;
                keyPressed = '';
            case "x"
                v_obj_des = [0;0];
                omega_obj_des = omega_obj_des_max*(-1);
                keyPressed = '';
            otherwise
                v_obj_des = [0;0];
                omega_obj_des = 0;
        end
        
        % position feedforward to slave
        slave.retrieve_poses();
        if isempty(slave.G)
            slave.swarm_syn(SYN_ID, s)
        else
            % if size(null(slave.G),2) > 2*slave.N-3 ...
            %         || ~inpolygon(0,0,cos(slave.robot_poses(3,slave.robots_in_contact_ids)),sin(slave.robot_poses(3,slave.robots_in_contact_ids)))
            % if ~positive_span(slave.get_grasp_theta(obj.o_(1:2,1:2)), 3)
            % if ~in_cone(slave.get_grasp_theta(obj.o_(1:2,1:2)), [v_obj_des;omega_obj_des])
            % slave.swarm_syn_and_opt_obj_manip(SYN_ID, s, zeros(2,1), 0, obj.o_(1:2,1:2))
            slave.swarm_syn_and_opt_obj_manip(SYN_ID, s, v_obj_des, omega_obj_des, obj.o_(1:2,1:2))
        end
        slave.step()
        
        % check and plot contact points
        [robot_idcs, contact_points, ~] = obj.check_contact(slave.robot_poses(1:2,:)', gcf);
        slave.update_grasp_matrix(robot_idcs, contact_points, obj.o_);
        
        obj.set_grasp_matrix(slave.G);           % set object grasp matrix
        obj.set_contact_points(contact_points);  % set object contact points
        
        % integrate object and restore robot positions
        contact_points_integrated_positions = obj.step(robot_idcs, slave.v, 10, 0); % this can easily simulate non-operational robots
        robot_integrated_positions = slave.robot_poses;
        robot_integrated_positions(1:2, robot_idcs) = contact_points_integrated_positions(:, robot_idcs);
        robot_integrated_positions = slave.check_robot_collisions(robot_integrated_positions);
        
        for i = 1 : slave.N
            % this needs some logic, e.g. if (contact changed || object pose changed)
            slave.set_theta_hinge(i, slave.robot_poses(3,i))
        end
        
        slave.overwrite_poses(robot_integrated_positions)
        
%         [robot_idcs, contact_points, ~] = obj.check_contact(slave.robot_poses(1:2,:)', gcf);
%         slave.update_grasp_matrix(robot_idcs, contact_points, obj.o_);
        
        % slave.plot_bb([0,1,0]);
        
%         gcf;
%         slave_frames = [slave_frames, getframe(gcf)];
        % force feedback to master
        % ---------------------------------------------------------- ACTUAL
        % master.set_forces(forces) % or show_forces(forces)
        % ------------------------------------------------------------ TEST
        % ... do nothing ...
        % ------------------------------------------------------------- END
    end
else % if (SIMULATED_MASTER)
    % while (1)
    %     % read master joints
    %     q = receive(jointsub,10);
    %
    %     % calculate synergies
    %     z = pinv(Syn.S(:,1:2))*q.Position
    %
    %     slave.move_synergy(1, z(1))
    % end
end

function b = positive_span(A,n)
b = inhull(zeros(1,n), A');
end

function in = in_cone(A,v)
in = false;
Ap = [];
for i = 1 : size(A,2)
    if A(:,i)'*v > 0
        Ap(:,end+1) = A(:,i);
    end
end
Ap
for i = 1 : size(Ap,2)
    for j = i+1 : size(Ap,2)
        if sign((Ap(1,i)*v(2)-Ap(2,i)*v(1))*(Ap(1,j)*v(2)-Ap(2,j)*v(1))) < 0
            in = true;
            return
        end
    end
end
end

function in = inhull(testpts,xyz,tess,tol)
% inhull: tests if a set of points are inside a convex hull
% usage: in = inhull(testpts,xyz)
% usage: in = inhull(testpts,xyz,tess)
% usage: in = inhull(testpts,xyz,tess,tol)
%
% arguments: (input)
%  testpts - nxp array to test, n data points, in p dimensions
%       If you have many points to test, it is most efficient to
%       call this function once with the entire set.
%
%  xyz - mxp array of vertices of the convex hull, as used by
%       convhulln.
%
%  tess - tessellation (or triangulation) generated by convhulln
%       If tess is left empty or not supplied, then it will be
%       generated.
%
%  tol - (OPTIONAL) tolerance on the tests for inclusion in the
%       convex hull. You can think of tol as the distance a point
%       may possibly lie outside the hull, and still be perceived
%       as on the surface of the hull. Because of numerical slop
%       nothing can ever be done exactly here. I might guess a
%       semi-intelligent value of tol to be
%
%         tol = 1.e-13*mean(abs(xyz(:)))
%
%       In higher dimensions, the numerical issues of floating
%       point arithmetic will probably suggest a larger value
%       of tol.
%
%       DEFAULT: tol = 0
%
% arguments: (output)
%  in  - nx1 logical vector
%        in(i) == 1 --> the i'th point was inside the convex hull.
%  
% Example usage: The first point should be inside, the second out
%
%  xy = randn(20,2);
%  tess = convhulln(xy);
%  testpoints = [ 0 0; 10 10];
%  in = inhull(testpoints,xy,tess)
%
% in = 
%      1
%      0
%
% A non-zero count of the number of degenerate simplexes in the hull
% will generate a warning (in 4 or more dimensions.) This warning
% may be disabled off with the command:
%
%   warning('off','inhull:degeneracy')
%
% See also: convhull, convhulln, delaunay, delaunayn, tsearch, tsearchn
%
% Author: John D'Errico
% e-mail: woodchips@rochester.rr.com
% Release: 3.0
% Release date: 10/26/06
% get array sizes
% m points, p dimensions
p = size(xyz,2);
[n,c] = size(testpts);
if p ~= c
  error 'testpts and xyz must have the same number of columns'
end
if p < 2
  error 'Points must lie in at least a 2-d space.'
end
% was the convex hull supplied?
if (nargin<3) || isempty(tess)
    try
        tess = convhulln(xyz);
    catch
        in = false;
        return
    end
end
[nt,c] = size(tess);
if c ~= p
  error 'tess array is incompatible with a dimension p space'
end
% was tol supplied?
if (nargin<4) || isempty(tol)
  tol = 0;
end
% build normal vectors
switch p
  case 2
    % really simple for 2-d
    nrmls = (xyz(tess(:,1),:) - xyz(tess(:,2),:)) * [0 1;-1 0];
    
    % Any degenerate edges?
    del = sqrt(sum(nrmls.^2,2));
    degenflag = (del<(max(del)*10*eps));
    if sum(degenflag)>0
      warning('inhull:degeneracy',[num2str(sum(degenflag)), ...
        ' degenerate edges identified in the convex hull'])
      
      % we need to delete those degenerate normal vectors
      nrmls(degenflag,:) = [];
      nt = size(nrmls,1);
    end
  case 3
    % use vectorized cross product for 3-d
    ab = xyz(tess(:,1),:) - xyz(tess(:,2),:);
    ac = xyz(tess(:,1),:) - xyz(tess(:,3),:);
    nrmls = cross(ab,ac,2);
    degenflag = false(nt,1);
  otherwise
    % slightly more work in higher dimensions, 
    nrmls = zeros(nt,p);
    degenflag = false(nt,1);
    for i = 1:nt
      % just in case of a degeneracy
      % Note that bsxfun COULD be used in this line, but I have chosen to
      % not do so to maintain compatibility. This code is still used by
      % users of older releases.
      %  nullsp = null(bsxfun(@minus,xyz(tess(i,2:end),:),xyz(tess(i,1),:)))';
      nullsp = null(xyz(tess(i,2:end),:) - repmat(xyz(tess(i,1),:),p-1,1))';
      if size(nullsp,1)>1
        degenflag(i) = true;
        nrmls(i,:) = NaN;
      else
        nrmls(i,:) = nullsp;
      end
    end
    if sum(degenflag)>0
      warning('inhull:degeneracy',[num2str(sum(degenflag)), ...
        ' degenerate simplexes identified in the convex hull'])
      
      % we need to delete those degenerate normal vectors
      nrmls(degenflag,:) = [];
      nt = size(nrmls,1);
    end
end
% scale normal vectors to unit length
nrmllen = sqrt(sum(nrmls.^2,2));
% again, bsxfun COULD be employed here...
%  nrmls = bsxfun(@times,nrmls,1./nrmllen);
nrmls = nrmls.*repmat(1./nrmllen,1,p);
% center point in the hull
center = mean(xyz,1);
% any point in the plane of each simplex in the convex hull
a = xyz(tess(~degenflag,1),:);
% ensure the normals are pointing inwards
% this line too could employ bsxfun...
%  dp = sum(bsxfun(@minus,center,a).*nrmls,2);
dp = sum((repmat(center,nt,1) - a).*nrmls,2);
k = dp<0;
nrmls(k,:) = -nrmls(k,:);
% We want to test if:  dot((x - a),N) >= 0
% If so for all faces of the hull, then x is inside
% the hull. Change this to dot(x,N) >= dot(a,N)
aN = sum(nrmls.*a,2);
% test, be careful in case there are many points
in = false(n,1);
% if n is too large, we need to worry about the
% dot product grabbing huge chunks of memory.
memblock = 1e6;
blocks = max(1,floor(n/(memblock/nt)));
aNr = repmat(aN,1,length(1:blocks:n));
for i = 1:blocks
   j = i:blocks:n;
   if size(aNr,2) ~= length(j),
      aNr = repmat(aN,1,length(j));
   end
   in(j) = all((nrmls*testpts(j,:)' - aNr) >= -tol,1)';
end
end

function teleop(src, event)
global keyPressed
keyPressed = event.Key;
end