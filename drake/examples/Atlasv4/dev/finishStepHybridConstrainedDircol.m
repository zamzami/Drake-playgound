function [p,v,xtraj,utraj,ltraj,z,F,info,traj_opt] = finishStepHybridConstrainedDircol(z0,xtraj,utraj,ltraj)
step_height = .1;

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
% options.terrain = RigidBodyFlatTerrain();
options.terrain = RigidBodyLinearStepTerrain(step_height,.35,.02);
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

N = [8,5];
% % N = [2,2,2,2,2];
% % N = N+1;
% % N = [7,3,3,3,7];
duration = {[.1 .5],[.1 .5]};
modes = {[3;4],[1;2;3;4]};

step_final = load ('data/step_final_state.mat');

x0 = step_final.xf_step;
xf = zeros(20,1);
 
to_options.mode_options{1}.active_inds = [1;2;4];
to_options.mode_options{2}.active_inds = [1;2;4;5;6;8];
to_options.non_penetration = false;

contact_q0 = x0(1:10);
contact_q0(1) = contact_q0(1) + 1;
to_options.mode_options{1}.contact_q0 =  contact_q0;
to_options.mode_options{2}.contact_q0 =  contact_q0;
to_options.mode_options{3}.contact_q0 =  contact_q0;
% to_options.mode_options{4}.contact_q0 =  contact_q0;
% to_options.mode_options{5}.contact_q0 =  contact_q0;


% N = N(1);
% modes = modes(1);
% duration = duration(1);
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
 
fn1 = drakeFunction.kinematic.WorldPosition(p,idxA(1),p.T_2D_to_3D'*xA(:,1),2);
fn2 = drakeFunction.kinematic.WorldPosition(p,idxA(2),p.T_2D_to_3D'*xA(:,2),2);
traj_opt = traj_opt.addModeStateConstraint(1,FunctionHandleConstraint(.1,inf,p.getNumPositions, @fn1.eval, 1), 2:N(1), 1:p.getNumPositions);
traj_opt = traj_opt.addModeStateConstraint(1,FunctionHandleConstraint(.1,inf,p.getNumPositions, @fn2.eval, 1), 2:N(1), 1:p.getNumPositions);

l0 = [0;897.3515;0;179.1489];


traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(x0,x0),1);
% traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(x0 - [.01*ones(10,1);.1*ones(10,1)],x0+[.01*ones(10,1);.1*ones(10,1)]),1);
% traj_opt = traj_opt.addModeStateConstraint(length(N),BoundingBoxConstraint(xf(3:10),xf(3:10)),N(end),3:10);
traj_opt = traj_opt.addModeStateConstraint(length(N),BoundingBoxConstraint(xf(11:20),xf(11:20)),N(end),11:20);

% match up leg joints
traj_opt = traj_opt.addModeStateConstraint(length(N),LinearConstraint(zeros(3,1),zeros(3,1),[eye(3) -eye(3)]),N(end),[4;5;6;8;9;10]);


t_init{1} = linspace(0,.4,N(1));
traj_init.mode{1}.x = PPTrajectory(foh(t_init{1},repmat(x0,1,N(1))));
traj_init.mode{1}.u = PPTrajectory(foh(t_init{1},randn(7,N(1))));
traj_init.mode{1}.l = PPTrajectory(foh(t_init{1},repmat([l0],1,N(1))));

if length(N) > 1
  t_init{2} = linspace(0,.2,N(2));
  traj_init.mode{2}.x = PPTrajectory(foh(t_init{2},repmat(x0,1,N(2))));
  traj_init.mode{2}.u = PPTrajectory(foh(t_init{2},randn(7,N(2))));
  traj_init.mode{2}.l = PPTrajectory(foh(t_init{2},repmat([l0;l0],1,N(2))));
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


%   traj_opt = traj_opt.addModeRunningCost(1,@foot_height_fun);
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
  
%   traj_opt = traj_opt.addModeStateConstraint(i,BoundingBoxConstraint(p.joint_limit_min,p.joint_limit_max),1:N(i),1:10);
  
  % bound joint velocities
  joint_vel_max = 3;
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
    phi0 = [.3;.3;.3;.3];
    K = 5;
    
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