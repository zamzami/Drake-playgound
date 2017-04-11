function [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = TrajOpt(xtraj,utraj,ltraj,ljltraj)
warning('off','Drake:RRigidBodyManipulatorigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.use_bullet=true;
options.ignore_self_collisions = true;
options.replace_cylinders_with_capsules = false; %These collision geometries were replaced by RigidBodyCapsule objects, as the cylinder contact geometry is less robust.
p = PlanarRigidBodyManipulator('disk.urdf',options);
% trajopt = ContactImplicitTrajectoryOptimization(p,[],[],[],10,[1 1]);

%todo: add joint limits, periodicity constraint

N = 10;
T = 5;
T0 = 3;

% 
% R_periodic(2:end,p.getNumStates+2:end) = -eye(p.getNumStates-1);
% 
% periodic_constraint = LinearConstraint(zeros(p.getNumStates,1),zeros(p.getNumStates,1),R_periodic);
%Rolling_constraint=LinearConstraint(zeros(p.getNumStates,1),zeros(p.getNumStates,1),ones(8,8));
%Rolling_constraint=LinearConstraint(0,0,[0 0 0 0 1 0 0 -0.015 0 0 0 0 1 0 0 -0.015])
%[0 0 0 0 1 0 0 -0.015]
% x0 = [0;0;1;zeros(15,1)];
% xf = [0;0;1;zeros(15,1)];
% x0 = [0;1;zeros(10,1)];
% xf = [.2;1;zeros(10,1)];
x0=zeros(p.getNumStates,1);
x0(2)=0.4;
xf=zeros(p.getNumStates,1);
xf(1)=5;
xf(2)=0.3;
%xf=[3;0;0;0;0;0;0;0;0;0;0;0]
%              base_x = 0.000000
%               base_z = 0.000000
%  base_relative_pitch = 0.000000
%                theta = 0.000000
%          arm_angle_1 = 0.010000
%          arm_angle_2 = 0.010000
%            base_xdot = 0.000000
%            base_zdot = 0.000000
% base_relative_pitchdot = 0.000000
%             thetadot = 0.500000
%       arm_angle_1dot = 0.000000
%       arm_angle_2dot = 0.000000
N2 = floor(N/10);

%if nargin < 2
  %Try to come up with a reasonable trajectory
  %x1 = [.3;1;pi/8-pi/16;pi/8;-pi/8;pi/8;zeros(6,1)];
  t_init = linspace(0,T0,N);
  %traj_init.x = PPTrajectory(foh(t_init,linspacevec(x0,xf,N)));
  x1=zeros(8,1);
traj_init.x = PPTrajectory(foh(t_init,[linspacevec(x0,x1,N2), linspacevec(x1,xf,N-N2)]));
traj_init.u = PPTrajectory(foh(t_init,randn(1,N)));
traj_init.l = PPTrajectory(foh(t_init,[repmat([zeros(4,1)],1,N2) repmat([1;1;1;0],1,N-N2)]));
%   traj_init.ljl = PPTrajectory(foh(t_init,zeros(p.getNumJointLimitConstraints,N)));
% else
%   t_init = xtraj.pp.breaks;
%   traj_init.x = xtraj;
%   traj_init.u = utraj;
%   traj_init.l = ltraj;
%   traj_init.ljl = ljltraj;
% end
T_span = [1 T];


x0_min = [0;0.3;0;0;-inf(4,1)];
x0_max = [0;1;0;0;inf(4,1)];
xf_min = [6;-inf(7,1)];
xf_max =[10; inf(7,1)];

to_options.nlcc_mode = 2;
to_options.lincc_mode = 1;
to_options.compl_slack = .01;
to_options.lincompl_slack = .001;
to_options.jlcompl_slack = .01;
to_options.lambda_mult = p.getMass*9.81*T0/N;
to_options.lambda_jl_mult = T0/N;
to_options.active_collision_options.terrain_only = false;


traj_opt = ContactImplicitTrajectoryOptimization(p,N,T_span,to_options);
traj_opt = traj_opt.addRunningCost(@running_cost_fun);
traj_opt = traj_opt.addStateConstraint(ConstantConstraint(x0),1);
traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(x0_min,x0_max),1);
traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(xf_min,xf_max),N);
%traj_opt.addStateConstraint(Rolling_constraint,{1, 2,3,4,5,6,7,8,9,10,11,12,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,45,46,47,48,49,50});
%traj_opt = traj_opt.addStateConstraint(Rolling_constraint,{[1 N]});
% traj_opt = traj_opt.setCheckGrad(true);
%snprint('snopt.out');
traj_opt = traj_opt.setSolver('snopt');
 traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',1000);
 traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',200000);
 traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',200000);
[xtraj,utraj,ltraj,ljltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);

function [f,df] = running_cost_fun(h,x,u)
  f = h*u'*u;
  df = [u'*u zeros(1,8) 2*h*u'];
end
%r = TimeSteppingRigidBodyManipulator(p,.001);
v = p.constructVisualizer;
v.axis = [-0.5 7 -0.5 2];

v.display_dt = .05;

v.playback(xtraj);
end
