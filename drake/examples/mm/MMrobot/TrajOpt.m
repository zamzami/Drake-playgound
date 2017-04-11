function [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = TrajOpt(xtraj,utraj,ltraj,ljltraj)

%function [p,xtraj,utraj,ltraj,ljltraj,z,F,info,infeasible_constraint_name,traj_opt] = TrajOpt(xtraj,utraj,ltraj,ljltraj)
warning('off','Drake:RRigidBodyManipulatorigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
global r 
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
%options.replace_cylinders_with_capsules = true;
options.replace_cylinders_with_spheres=true;
options.use_bullet=true;
%options.twoD=true;%These collision geometries were replaced by RigidBodyCapsule objects, as the cylinder contact geometry is less robust.
p =PlanarRigidBodyManipulator('disk.urdf',options);
%p = TimeSteppingRigidBodyManipulator('disk.urdf',.01,options);
% trajopt = ContactImplicitTrajectoryOptimization(p,[],[],[],10,[1 1]);
%todo: add joint limits, periodicity constraint
%fcn=DrakeFunction(p.getStateFrame(1),p.getStateFrame(2))
%p.addPositionEqualityConstraint(DrakeFunctionConstraint(0,0,fcn))
N = 10;
T = 5;
T0 = 3;

% periodic constraint
% R_periodic = zeros(p.getNumStates,2*p.getNumStates);
% R_periodic(2,2) = 1; %z
% R_periodic(3,3) = 1; %pitch-hip w/symmetry
% R_periodic(3,5) = 1; %pitch-hip w/symmetry
% R_periodic(4,6) = 1; %knee w/symmetry
% R_periodic(6,4) = 1; %knee w/symmetry
% R_periodic(5,5) = -1; %hip w/symmetry
% 
% R_periodic(7,7) = 1; %x-vel
% R_periodic(8,8) = 1; %z-vel
% R_periodic(9,9) = 1; %pitch-hip w/symmetry
% R_periodic(9,11) = 1; %pitch-hip w/symmetry
% R_periodic(10,12) = 1; %knee w/symmetry
% R_periodic(12,10) = 1; %knee w/symmetry
% R_periodic(11,11) = -1; %hip w/symmetry
% 
% R_periodic(2:end,p.getNumStates+2:end) = -eye(p.getNumStates-1);
% 
% periodic_constraint = LinearConstraint(zeros(p.getNumStates,1),zeros(p.getNumStates,1),R_periodic);
%Rolling_constraint=LinearConstraint(zeros(p.getNumStates,1),zeros(p.getNumStates,1),ones(8,8));
Rolling_constraint=LinearConstraint(0,0,[1 ,-0.3 ]);
Rolling_constraint=Rolling_constraint.setName({'RC'});
Height_constraint=LinearConstraint(0.25,0.5,[1]);
%prog = prog.addConstraint(LinearConstraint(0,0,[cos(2*rw.alpha),-1]),[prog.x_inds(2,N);prog.x_inds(2,1)]);
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
%xf(2)=0.5;
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
  x1 = [0;0.3;0;0;0;0;0;0];
  t_init = linspace(0,T0,N);
  traj_init.x = PPTrajectory(foh(t_init,linspacevec(x0,xf,N)));
  %traj_init.x = PPTrajectory(foh(t_init,[linspacevec(x0,x1,1), linspacevec(x1,xf,N-1)]));
traj_init.x = PPTrajectory(foh(t_init,[linspacevec(x0,x1,N2), linspacevec(x1,xf,N-N2)]));
traj_init.u = PPTrajectory(foh(t_init,[zeros(1,N2), 10*randn(1,N-N2)]));
%traj_init.u = PPTrajectory(foh(t_init,randn(1,N)));
 traj_init.l = PPTrajectory(foh(t_init,[repmat([1;zeros(3,1)],1,N2) repmat([1;1;1;0],1,N-N2)]));
%traj_init.l = PPTrajectory(foh(t_init,[repmat([0;0;0;0],1,1), repmat([1;0;1;0],1,1), repmat([1;0;1;0],1,3)]));
  % traj_init.ljl = PPTrajectory(foh(t_init,zeros(p.getNumJointLimitConstraints,N)));
% else
%   t_init = xtraj.pp.breaks;
%   traj_init.x = xtraj;
%   traj_init.u = utraj;
%   traj_init.l = ltraj;
%   traj_init.ljl = ljltraj;
% end
T_span = [1 T];


x0_min = [0;0.3;0;0;0;0;0;0];
x0_max = [0;1;0;0;0;0;0;0];
xf_min = [5;0.3;-inf(6,1)];
xf_max =[10;1; inf(6,1)];

%xf_min = [12;0];
%xf_max =[20;100];

to_options.nlcc_mode = 2;
to_options.lincc_mode = 1;
to_options.compl_slack = .0001;
to_options.lincompl_slack = .0001;
to_options.jlcompl_slack = .01;
to_options.lambda_mult =p.getMass*9.81*T0/N;
to_options.lambda_jl_mult = T0/N;
to_options.active_collision_options.terrain_only = true;
to_options.constraint_err_tol=1e-4; 


traj_opt = ContactImplicitTrajectoryOptimization(p,N,T_span,to_options);
traj_opt = traj_opt.addRunningCost(@running_cost_fun);
%traj_opt = traj_opt.addFinalCost(@finalCost);
c0=ConstantConstraint(x0);
c0=c0.setName([{'c01'} ;{'c02'};  {'c03'}; {'c04'}; {'c05' };{'c06' };{'c07' };{'c08' }]);
% cnstr_name = cell(N,1);
% t_idx=[1:N];
% for i = 1:N
%     cnstr_name{i} = sprintf('arm_usy_symmetry[%d]',t_idx(i))
% end

Z= cell(1,N);
for k = 1:N
    Z{k} = k;
end
Z;

Bx0=BoundingBoxConstraint(x0_min,x0_max);
Bx0=Bx0.setName([{'bx0'};{'bx0'};{'bx0'};{'bx0'};{'bx0'};{'bx0'};{'bx0'};{'bx0'}])
Bxf=BoundingBoxConstraint(xf_min,xf_max)
Bxf=Bxf.setName([{'bxf'};{'bxf'};{'bxf'};{'bxf'};{'bxf'};{'bxf'};{'bxf'};{'bxf'}])
traj_opt = traj_opt.addStateConstraint(c0,1);
%traj_opt = traj_opt_con.setName(cnstr_name)
%traj_opt = traj_opt.addStateConstraint(ConstantConstraint(xf),N);
traj_opt = traj_opt.addStateConstraint(Bx0,1);
traj_opt = traj_opt.addStateConstraint(Bxf,N);
%traj_opt.addStateConstraint(Rolling_constraint,{[1, 2,3,4,5,6,7,8,9,10,11,12,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,45,46,47,48,49,50]});
%traj_opt.addStateConstraint(Rolling_constraint,{[2,3,4,5,6,7,8,9,10,]});
traj_opt = traj_opt.addStateConstraint(Rolling_constraint,Z,[5;7]);
traj_opt = traj_opt.addStateConstraint(Height_constraint,Z,[2]);
traj_opt =traj_opt.addTrajectoryDisplayFunction(@(t,x,u)plotDircolTraj(t,x,u,[1 2]));
%traj_opt =traj_opt.addTrajectoryDisplayFunction(@(t,x,u)plotDircolTraj(t,x,u,[4 8]));
 traj_opt = traj_opt.setCheckGrad(true);
snprint('snopt.out');
traj_opt = traj_opt.setSolver('snopt');
 traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',1000000);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',20000000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',50000000);
 traj_opt = traj_opt.setSolverOptions('snopt','MajorOptimalityTolerance',1e-02);
  traj_opt = traj_opt.setSolverOptions('snopt','MajorFeasibilityTolerance',1e-6);
 traj_opt = traj_opt.setSolverOptions('snopt','MinorFeasibilityTolerance',1e-06);
  traj_opt = traj_opt.setSolverOptions('snopt',' LinesearchTolerance',0.9);
 %traj_opt = traj_opt.setSolverOptions('snopt','sense ','Feasible point');
 %traj_opt = traj_opt.setconstraint_err_tol(1)

[xtraj,utraj,ltraj,ljltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);

%[xtraj,utraj,ltraj,ljltraj,z,F,info,infeasible_constraint_name] = traj_opt.solveTraj(t_init,traj_init);

function [f,df] = running_cost_fun(h,x,u)
  f = h*u'*u;
  df = [u'*u zeros(1,8) 2*h*u'];
end

% function [f, df] = running_cost_fun(h,x,u)
%     %h=-10;
%     q=zeros(1,8);
%     q(5)=10;
%     Q=diag(q);
%     xnom=[x(1);x(2);x(3);x(4);-0.3*x(8);x(6);x(7);x(8)]
%     xbar=x-xnom
%   f = h*u'*u+xbar'*Q*xbar;
%   df = [u'*u 2*xbar'*Q 2*h*u'];
% end

function [h,dh] = finalCost(t,x)
  h = t;
  dh = [1,zeros(1,size(x,1))];
end

function plotDircolTraj(t,x,u,plotdims)
  figure(27);
  h=plot(x(plotdims(1),:),x(plotdims(2),:),'r.-');
  drawnow;
  %delete(h);
end
%r = TimeSteppingRigidBodyManipulator(p,.001);
v = p.constructVisualizer;
v.axis = [-0.5 7 -0.5 2];

v.display_dt = .05;

v.playback(xtraj);
end

