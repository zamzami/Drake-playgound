function [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = TrajOptM(xtraj,utraj,ltraj,ljltraj)
warning('off','Drake:RRigidBodyManipulatorigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
global r 
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
options.replace_cylinders_with_capsules = false;
%options.twoD=true;%These collision geometries were replaced by RigidBodyCapsule objects, as the cylinder contact geometry is less robust.
p =PlanarRigidBodyManipulator('mm.urdf',options);
%p = TimeSteppingRigidBodyManipulator('disk.urdf',.01,options);
% trajopt = ContactImplicitTrajectoryOptimization(p,[],[],[],10,[1 1]);
%todo: add joint limits, periodicity constraint
%fcn=DrakeFunction(p.getStateFrame(1),p.getStateFrame(2))
%p.addPositionEqualityConstraint(DrakeFunctionConstraint(0,0,fcn))
N = 10;
T = 10;
T0 = 10;


%Rolling_constraint=LinearConstraint(zeros(p.getNumStates,1),zeros(p.getNumStates,1),ones(8,8));
Rolling_constraint=LinearConstraint(0,0,[1 ,-0.3 ]);
Rolling_constraint=Rolling_constraint.setName({'RC'});
%prog = prog.addConstraint(LinearConstraint(0,0,[cos(2*rw.alpha),-1]),[prog.x_inds(2,N);prog.x_inds(2,1)]);
%[0 0 0 0 1 0 0 -0.015]
% x0 = [0;0;1;zeros(15,1)];
% xf = [0;0;1;zeros(15,1)];
% x0 = [0;1;zeros(10,1)];
% xf = [.2;1;zeros(10,1)];
x0=zeros(p.getNumStates,1);
x0(2)=0.3;
xf=zeros(p.getNumStates,1);
xf(1)=5;
xf(2)=1.0;

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
  x1 = zeros(p.getNumStates,1);
  x1(3)=0.3;
  t_init = linspace(0,T0,N);
  traj_init.x = PPTrajectory(foh(t_init,linspacevec(x0,xf,N)));
  %traj_init.x = PPTrajectory(foh(t_init,[linspacevec(x0,x1,1), linspacevec(x1,xf,N-1)]));
traj_init.x = PPTrajectory(foh(t_init,[linspacevec(x0,x1,N2), linspacevec(x1,xf,N-N2)]));
%traj_init.u = PPTrajectory(foh(t_init,[zeros(3,N2), randn(3,N-N2)]));
%traj_init.u = PPTrajectory(foh(t_init,randn(1,N)));
 %traj_init.l = PPTrajectory(foh(t_init,[repmat([1;zeros(11,1)],1,N2) repmat([zeros(4,1);1;zeros(3,1)],1,N-N2)]));
traj_init.l = PPTrajectory(foh(t_init,[repmat([0;0;0;0;zeros(8,1)],1,1), repmat([1;0;0;0;zeros(8,1)],1,2), repmat([1;1;1;0;zeros(8,1)],1,4),repmat([0;0;0;0;zeros(8,1)],1,3)]));
  % traj_init.ljl = PPTrajectory(foh(t_init,zeros(p.getNumJointLimitConstraints,N)));
% else
%   t_init = xtraj.pp.breaks;
%   traj_init.x = xtraj;
%   traj_init.u = utraj;
%   traj_init.l = ltraj;
%   traj_init.ljl = ljltraj;
% end
T_span = [1 T];


x0_min =zeros(p.getNumStates,1);
x0_min(2)=0.3;
x0_max =zeros(p.getNumStates,1);
x0_max(2)=1;
xf_min = [5;0.7;-inf(10,1)];
xf_max =[10;3; inf(10,1)];

%xf_min = [12;0];
%xf_max =[20;100];

to_options.nlcc_mode = 2;
to_options.lincc_mode = 1;
to_options.compl_slack = .0001;
to_options.lincompl_slack = .0001;
to_options.jlcompl_slack = .01;
to_options.lambda_mult =p.getMass*9.81*T0/N;
to_options.lambda_jl_mult = T0/N;
to_options.active_collision_options.terrain_only = false;
to_options.constraint_err_tol=1e-3; 
to_options.cstr_name={'NLC'};


traj_opt = ContactImplicitTrajectoryOptimization(p,N,T_span,to_options);
traj_opt = traj_opt.addRunningCost(@running_cost_fun);
%traj_opt = traj_opt.addFinalCost(@finalCost);
c0=ConstantConstraint(x0);
c0=c0.setName([{'c01'} ;{'c02'};  {'c03'}; {'c04'}; {'c05' };{'c06' };{'c07' };{'c08' };{'c09' };{'c010' };{'c011' };{'c012' }]);
% cnstr_name = cell(N,1);
% t_idx=[1:N];
% for i = 1:N
%     cnstr_name{i} = sprintf('arm_usy_symmetry[%d]',t_idx(i))
% end

Z= cell(1,N);
for k = 1:N
    Z{k} = k;
end
Z

Bx0=BoundingBoxConstraint(x0_min,x0_max);
Bx0=Bx0.setName([{'bx01'};{'bx02'};{'bx03'};{'bx04'};{'bx05'};{'bx06'};{'bx07'};{'bx08'};{'bx09'};{'bx010'};{'bx011'};{'bx012'}])
Bxf=BoundingBoxConstraint(xf_min,xf_max)
Bxf=Bxf.setName([{'bxf'};{'bxf'};{'bxf'};{'bxf'};{'bxf'};{'bxf'};{'bxf'};{'bxf'};{'bxf'};{'bxf'};{'bxf'};{'bxf'}])
traj_opt = traj_opt.addStateConstraint(c0,1);
%traj_opt = traj_opt_con.setName(cnstr_name)
%traj_opt = traj_opt.addStateConstraint(ConstantConstraint(xf),N);
traj_opt = traj_opt.addStateConstraint(Bx0,1);
traj_opt = traj_opt.addStateConstraint(Bxf,N);
%traj_opt.addStateConstraint(Rolling_constraint,{[1, 2,3,4,5,6,7,8,9,10,11,12,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,45,46,47,48,49,50]});
%traj_opt.addStateConstraint(Rolling_constraint,{[2,3,4,5,6,7,8,9,10,]});
traj_opt = traj_opt.addStateConstraint(Rolling_constraint,Z,[5;7])
traj_opt =traj_opt.addTrajectoryDisplayFunction(@(t,x,u)plotDircolTraj(t,x,u,[1 2]));
%traj_opt =traj_opt.addTrajectoryDisplayFunction(@(t,x,u)plotDircolTraj(t,x,u,[4 8]));
 %traj_opt = traj_opt.setCheckGrad(true);
snprint('snopt.out');
traj_opt = traj_opt.setSolver('snopt');
 traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',1000000);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',2000000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',5000000);
 traj_opt = traj_opt.setSolverOptions('snopt','MajorOptimalityTolerance',1e-02);
  traj_opt = traj_opt.setSolverOptions('snopt','MajorFeasibilityTolerance',1e-04);
 traj_opt = traj_opt.setSolverOptions('snopt','MinorFeasibilityTolerance',1e-04);
  traj_opt = traj_opt.setSolverOptions('snopt',' LinesearchTolerance',0.9);
 %traj_opt = traj_opt.setSolverOptions('snopt','sense ','Feasible point');
 %traj_opt = traj_opt.setconstraint_err_tol(1e-06);

tic
[xtraj,utraj,ltraj,ljltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);
toc
function [f, df] = running_cost_fun(h,x,u)
    %h=-10;
    q=zeros(1,12);
    q(5)=100;
    Q=diag(q);
    xnom=[x(1);x(2);x(3);x(4);-0.3*x(8);x(6);x(7);x(8);x(9);x(10);x(11);x(12)]
    xbar=x-xnom
  f = h*u'*u+xbar'*Q*xbar;
  df = [u'*u 2*xbar'*Q 2*h*u'];
end

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
v.axis = [-0.5 7 -0.5 4];

v.display_dt = .05;

v.playback(xtraj);
end
