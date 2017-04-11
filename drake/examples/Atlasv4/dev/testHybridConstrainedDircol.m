function [p,v,xtraj,utraj,ltraj,z,F,info,traj_opt] = testHybridConstrainedDircol(z0,xtraj,utraj,ltraj)

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
options.use_new_kinsol = true;

p = PlanarRigidBodyManipulator('../urdf/atlas_simple_planar_contact.urdf',options);
nq = p.getNumPositions;
nv = p.getNumVelocities;
nu = p.getNumInputs;
nx = p.getNumStates();
v = p.constructVisualizer();
%works with this
% N = [10,5,5];
% duration = {[.2 .5],[.02 .2],[.02 .2]};
% modes = {[1;2],[1;2;3],[1;2;3;4]};

% % works with this
% N = [5,3,3,5];
% duration = {[.2 .5],[.02 .2],[.02 .2],[.02 .2]};
% modes = {[1;2],[1;2;3],[1;2;3;4],[2;3;4]};

% works with this, before increasing N
N = [6,3,3,5,3];

N = [6,5,5,5,5];

N = [7,5,5,5,5];
% % N = [2,2,2,2,2];
% % N = N+1;
% % N = [7,3,3,3,7];
duration = {[.2 .7],[.05 .2],[.05 .2],[.05 .5], [.1 .7]};
modes = {[1;2],[1;2;3],[1;2;3;4],[2;3;4], [3;4]};



x0 = [      0
    0.9371
    0.2000
   -0.4414
    0.2625
   -0.0211
    0.0891
   -0.4997
    0.9273
   -0.6403
    0.2346
   -0.0077
    0.0731
   -0.2012
    0.7876
   -0.6596
    0.1798
   -1.9375
    2.5602
   -2.2319]; 
to_options.lambda_bound = 20; %was 400
to_options.mode_options{1}.active_inds = [1;2;4];
to_options.mode_options{2}.active_inds = [1;2;4;5;6];
% to_options.mode_options{3} = struct();
% to_options.mode_options{4} = struct();
% to_options.mode_options{5} = struct();
to_options.mode_options{3}.active_inds = [1;2;4;5;6;8];
to_options.mode_options{4}.active_inds = [1;2;3;4;6];
to_options.mode_options{5}.active_inds = [1;2;4];

traj_opt = ConstrainedHybridTrajectoryOptimization(p,modes,N,duration,to_options);


% Add foot height constraint
[~,normal,d,xA,xB,idxA,idxB,mu,n,D] = contactConstraints(p,x0(1:p.getNumPositions));
if idxA(1) == 1,
  tmp = idxA;
  idxA = idxB;
  idxB = tmp;
  tmp = xA;
  xA = xB;
  xB = tmp;
end
assert(isequal(idxB,ones(size(idxB))))
 
fn1 = drakeFunction.kinematic.WorldPosition(p,idxA(3),p.T_2D_to_3D'*xA(:,3),2);
fn2 = drakeFunction.kinematic.WorldPosition(p,idxA(4),p.T_2D_to_3D'*xA(:,4),2);
traj_opt = traj_opt.addModeStateConstraint(1,FunctionHandleConstraint(.1,inf,p.getNumPositions, @fn1.eval, 1), floor(N(1)/2), 1:p.getNumPositions);
traj_opt = traj_opt.addModeStateConstraint(1,FunctionHandleConstraint(.1,inf,p.getNumPositions, @fn2.eval, 1), floor(N(1)/2), 1:p.getNumPositions);

for i=floor(N(1)/2)+1:N(1)-1,
traj_opt = traj_opt.addModeStateConstraint(1,FunctionHandleConstraint(.03,inf,p.getNumPositions, @fn1.eval, 1), i, 1:p.getNumPositions);
traj_opt = traj_opt.addModeStateConstraint(1,FunctionHandleConstraint(.07,inf,p.getNumPositions, @fn2.eval, 1), i, 1:p.getNumPositions);  
end


l0 = [0;897.3515;0;179.1489];

traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(x0 - [0;.1*ones(9,1);.1*ones(10,1)],x0+[0;.1*ones(9,1);.1*ones(10,1)]),1);
% traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(x0 - [.01*ones(10,1);.1*ones(10,1)],x0+[.01*ones(10,1);.1*ones(10,1)]),1);
traj_opt = traj_opt.addModeStateConstraint(length(N),BoundingBoxConstraint(.3,inf),N(end),1);

t_init{1} = linspace(0,.4,N(1));
traj_init.mode{1}.x = PPTrajectory(foh(t_init{1},repmat(x0,1,N(1))));
traj_init.mode{1}.u = PPTrajectory(foh(t_init{1},randn(7,N(1))));
traj_init.mode{1}.l = PPTrajectory(foh(t_init{1},repmat(l0,1,N(1))));

if length(N) > 1
  t_init{2} = linspace(0,.2,N(2));
  traj_init.mode{2}.x = PPTrajectory(foh(t_init{2},repmat(x0,1,N(2))));
  traj_init.mode{2}.u = PPTrajectory(foh(t_init{2},randn(7,N(2))));
  traj_init.mode{2}.l = PPTrajectory(foh(t_init{2},repmat([l0;l0(1:2)],1,N(2))));
end
if length(N) > 2
  t_init{3} = linspace(0,.2,N(3));
  traj_init.mode{3}.x = PPTrajectory(foh(t_init{3},repmat(x0,1,N(3))));
  traj_init.mode{3}.u = PPTrajectory(foh(t_init{3},randn(7,N(3))));
  traj_init.mode{3}.l = PPTrajectory(foh(t_init{3},repmat([l0;l0],1,N(3))));
end
if length(N) > 3
  t_init{4} = linspace(0,.2,N(4));
  traj_init.mode{4}.x = PPTrajectory(foh(t_init{4},repmat(x0,1,N(4))));
  traj_init.mode{4}.u = PPTrajectory(foh(t_init{4},randn(7,N(4))));
  traj_init.mode{4}.l = PPTrajectory(foh(t_init{4},repmat([l0;l0(1:2)],1,N(4))));
end
if length(N) > 4
  t_init{5} = linspace(0,.2,N(5));
  traj_init.mode{5}.x = PPTrajectory(foh(t_init{5},repmat(x0,1,N(5))));
  traj_init.mode{5}.u = PPTrajectory(foh(t_init{5},randn(7,N(5))));
  traj_init.mode{5}.l = PPTrajectory(foh(t_init{5},repmat([l0],1,N(5))));
  
  % build periodic constraint matrix

  R_periodic = zeros(p.getNumStates,2*p.getNumStates);
  R_periodic(2,2) = 1; %z
  R_periodic(3,3) = 1; %pitch
  R_periodic(4:6,8:10) = eye(3); %leg joints w/symmetry
  R_periodic(8:10,4:6) = eye(3); %leg joints w/symmetry
  R_periodic(7,7) = 1; % back joint
  R_periodic(11:13,11:13) = eye(3); %x,z,pitch velocities
  R_periodic(14:16,18:20) = eye(3); %leg joints w/symmetry
  R_periodic(18:20,14:16) = eye(3); %leg joints w/symmetry
  R_periodic(17,17) = 1; % back joint
  R_periodic(2:end,p.getNumStates+2:end) = -eye(p.getNumStates-1);
  
  periodic_constraint = LinearConstraint(zeros(nx-1,1),zeros(nx-1,1),R_periodic(2:end,:));
  periodic_inds = [traj_opt.mode_opt{1}.x_inds(:,1) + traj_opt.var_offset(1);...
    traj_opt.mode_opt{end}.x_inds(:,end) + traj_opt.var_offset(end)];
  traj_opt = traj_opt.addConstraint(periodic_constraint,periodic_inds);
end

if nargin > 1
  for i=1:length(N)
    t_init{i} = linspace(0,diff(xtraj{i}.tspan),N(i));
    traj_init.mode{i}.x = xtraj{i};
    if nargin < 2
      traj_init.mode{i}.u = utraj{i};
    end
    if nargin > 3
      traj_init.mode{i}.l = ltraj{i};
    end
  end
end

traj_opt = traj_opt.setSolverOptions('snopt','print','snopt.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',200);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',50000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',2000000);


traj_opt = traj_opt.addModeRunningCost(1,@foot_height_fun);
% traj_opt = traj_opt.setCheckGrad(true);
for i=1:length(N)
  traj_opt = traj_opt.addModeRunningCost(i,@running_cost_fun);
  traj_opt = traj_opt.addModeRunningCost(i,@pelvis_motion_fun);
  
end
% traj_opt = traj_opt.addModeRunningCost(2,@running_cost_fun);
% traj_opt = traj_opt.addModeRunningCost(3,@running_cost_fun);
% traj_opt = traj_opt.addModeRunningCost(4,@running_cost_fun);
% traj_opt = traj_opt.addFinalCost(@final_cost_fun);

for i=1:length(N)
  knee_inds = traj_opt.mode_opt{i}.x_inds([5;9],:) + traj_opt.var_offset(i);
  knee_inds = knee_inds(:);
  knee_constraint = BoundingBoxConstraint(.1*ones(length(knee_inds),1),inf(length(knee_inds),1));
%   traj_opt = traj_opt.addBoundingBoxConstraint(knee_constraint,knee_inds);
  
  traj_opt = traj_opt.addModeStateConstraint(i,BoundingBoxConstraint(p.joint_limit_min,p.joint_limit_max),1:N(i),1:10);
  
  % bound joint velocities
  joint_vel_max = 10;
  joint_vel_bound = BoundingBoxConstraint(-joint_vel_max*ones(p.getNumVelocities,1),joint_vel_max*ones(p.getNumVelocities,1));
  traj_opt = traj_opt.addModeStateConstraint(i,joint_vel_bound,1:N(i),11:20);
end

traj_opt = traj_opt.compile();
if nargin == 1
  [xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTrajFromZ(z0);
else
  [xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);
end


  function [f,df] = running_cost_fun(h,x,u)
    R = .1;
    f = h*u'*R*u;
    df = [u'*R*u zeros(1,20) 2*h*u'*R];    
  end

  function [f,df] = pelvis_motion_fun(h,x,u)
    nu = length(u);  
    pitch_idx = 3;
    pitch_dot_idx = p.getNumPositions+pitch_idx;

    Kq = 50;
    Kqd = 50;
    f = Kq*x(pitch_idx)^2 + Kqd*x(pitch_dot_idx)^2; % try to avoid moving the pelvis quickly
    df = zeros(1,1+nx+nu);
    df(1+pitch_idx) = 2*Kq*x(pitch_idx);
    df(1+pitch_dot_idx) = 2*Kqd*x(pitch_dot_idx);
    
    df = df*h;
    df(1) = f;
    f = f*h;
    
%    K = 100;
%    K_log = 100;
%    f = sum(-K*log(K_log*phi + .2));
%    df = [0 sum(-K*K_log*n./(K_log*repmat(phi,1,length(q)) + .2)) zeros(1,15)];
  end

  function [f,df] = foot_height_fun(h,x,u)
    q = x(1:nq);
    
    [phi,~,~,~,~,~,~,~,n] = p.contactConstraints(q,false,struct('terrain_only',true));
    phi0 = [.2;.2;.2;.2];
    K = 50;
    
%     [~,I1] = min(phi(1:2));
%     [~,I2] = min(phi(3:4));
%     phi = [phi(I1);phi(2+I2)];
%     n = [n(I1,:);n(2+I2,:)];    
%     phi0 = [.2;.2];
    
    I = find(phi < phi0);
    f = K*(phi(I) - phi0(I))'*(phi(I) - phi0(I));
    % phi: 2x1
    % n: 2xnq
    df = [0 2*K*(phi(I)-phi0(I))'*n(I,:) zeros(1,nv+nu)];
    
    df = df*h;
    df(1) = f;
    f = f*h;
    
    
%     K = 500;
%     f = K*sum(phi0(I) - phi(I));
%     df = [0 K*sum(n(I,:),1) zeros(1,nv+nu)];

%    K = 100;
%    K_log = 100;
%    f = sum(-K*log(K_log*phi + .2));
%    df = [0 sum(-K*K_log*n./(K_log*repmat(phi,1,length(q)) + .2)) zeros(1,15)];
  end

end