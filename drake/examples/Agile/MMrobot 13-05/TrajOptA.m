function [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = TrajOptA(xtraj,utraj,ltraj,ljltraj)
%warning('off','Drake:RRigidBodyManipulatorigidBodyManipulator:UnsupportedContactPoints');
%warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
%% Plant Settings

options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
options.replace_cylinders_with_spheres= true;
options.use_bullet=false;
%options.twoD=true;

p =PlanarRigidBodyManipulator('mmarm.urdf',options);

height=getBody(p,3);
height.joint_limit_min=0;

pitch=getBody(p,4);
pitch.joint_limit_min=0;
pitch.joint_limit_max=0;

arm1=getBody(p,6);
arm1.joint_limit_min=-pi/2;
arm1.joint_limit_max=pi/2;

arm2=getBody(p,7);
arm2.joint_limit_min=-pi/2;
arm2.joint_limit_max=pi/2;

p=setBody(p,4,pitch);
p=setBody(p,6,arm1);
p=setBody(p,7,arm2);

p= compile(p);

%p = p.addGeometryToBody('wheel',RigidBodySphere(0.8),'groupA');
%p=p.active_collision_options.collision_groups = {'groupA'};
%p= compile(p);

%%  Time paramters
N =100;
T = 10;
T0 =10;

%% Extra Constraints

%Rolling_constraint=LinearConstraint(zeros(p.getNumStates,1),zeros(p.getNumStates,1),ones(8,8));
Rolling_constraint=LinearConstraint(0,0,[1 ,-0.5]);
Rolling_constraint=Rolling_constraint.setName({'RC'});
%input_constraint=LinearConstraint(0,0,[1 ,-1]);
%input_constraint=input_constraint.setName({'IC'});
height_constraint=LinearConstraint(0.5,0.5,1);
height_constraint=height_constraint.setName({'ZC'});

pitch_constraint=LinearConstraint(0,0,1);
pitch_constraint=height_constraint.setName({'PC'});

%prog = prog.addConstraint(LinearConstraint(0,0,[cos(2*rw.alpha),-1]),[prog.x_inds(2,N);prog.x_inds(2,1)]);

%% State Trajectory Initailization
wheel_radius= 0.5;
final_distance=10;
final_rolling_angle=final_distance/wheel_radius;

x0=zeros(p.getNumStates,1);
x0(2)=0.6; % Initial height
x0(5)=pi/4; %inital joint angle for Arm 1
x0(6)=pi/4; % initial joint angle for Arm 2

xf=zeros(p.getNumStates,1);
xf(1)=final_distance; 
xf(2)=wheel_radius;
xf(3)=0; %zero pitch 
xf(4)=final_rolling_angle;
xf(5)=pi/4;  %final joint angle for Arm 1
xf(6)=pi/4; % final joint angle for Arm 2
xf(7)=0; %base_xdot
xf(8)=0; %base_zdot
xf(9)=0;   %base_relative_pitchdot
xf(10)=0; %thetadot
xf(11)=0; %arm_angle_1dot
xf(12)=0; %arm_angle_2dot

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


N2 = floor(0.1*N);


  %Try to come up with a reasonable trajectory
  x1 = x0;
  x1(2)=wheel_radius;
  t_init = linspace(0,T0,N);
  %traj_init.x = PPTrajectory(foh(t_init,linspacevec(x0,xf,N)));
  %traj_init.x = PPTrajectory(foh(t_init,[linspacevec(x0,x1,1), linspacevec(x1,xf,N-1)]));
traj_init.x = PPTrajectory(foh(t_init,[linspacevec(x0,x1,N2), linspacevec(x1,xf,N-N2)]));

%traj_init.u = PPTrajectory(foh(t_init,[linspacevec(0,1,N) ]));
%traj_init.u = PPTrajectory(foh(t_init,[zeros(1,N2), 10*ones(1,N-N2) ]));
%traj_init.u = PPTrajectory(foh(t_init,randn(1,N)));

%traj_init.l = PPTrajectory(foh(t_init,[repmat([zeros(4,1);zeros(4,1)],1,N2) repmat([1;1;0;0;1;1;0;0],1,N-N2)]));
%traj_init.l = PPTrajectory(foh(t_init,[repmat([0;0;0;0],1,1), repmat([1;0;1;0],1,1), repmat([1;0;1;0],1,3)]));
traj_init.l = PPTrajectory(foh(t_init,[repmat([0;0;0;0;zeros(4,1)],1,N2), repmat([1;1;0;0;zeros(4,1)],1,N-N2)]));

%traj_init.ljl = PPTrajectory(foh(t_init,zeros(p.getNumJointLimitConstraints,N)));

T_span = [1 T];

x0_min = [0;0.5;zeros(10,1)];
x0_max = [0;0.7;zeros(10,1)];
xf_min = [4;0.4;-inf(10,1)];
xf_max =[6;0.6; inf(10,1)];

%xf_min = [12;0];
%xf_max =[20;100];
%% Trajectory Optimisation
to_options.nlcc_mode = 2;
to_options.lincc_mode = 1;
to_options.compl_slack = 0.00001;%.001;
to_options.lincompl_slack =0.00001;% .001;
to_options.jlcompl_slack = .01;
to_options.lambda_mult =p.getMass*9.81;%*T0/N;
to_options.lambda_jl_mult = T0/N;
to_options.cstr_name={'NLC'};
%to_options.time_option=2;


traj_opt = ContactImplicitTrajectoryOptimization(p,N,T_span,to_options);
traj_opt = traj_opt.addRunningCost(@running_cost_fun);
%traj_opt = traj_opt.addFinalCost(@final_cost_fun);
%traj_opt = traj_opt.addFinalCost(@finalCost);
%c0=ConstantConstraint(x0);
%c0=c0.setName([{'c01'} ;{'c02'};  {'c03'}; {'c04'}; {'c05' };{'c06' };{'c07' };{'c08' }]);
% cnstr_name = cell(N,1);
% t_idx=[1:N];
% for i = 1:N
%     cnstr_name{i} = sprintf('arm_usy_symmetry[%d]',t_idx(i))
% end

Z= cell(1,N);
for k = 1:N
    Z{k} = k;
end


Bx0=BoundingBoxConstraint(x0_min,x0_max);
%Bx0=Bx0.setName([{'bx0'};{'bx0'};{'bx0'};{'bx0'};{'bx0'};{'bx0'};{'bx0'};{'bx0'}]);
Bxf=BoundingBoxConstraint(xf_min,xf_max);
%Bxf=Bxf.setName([{'bxf'};{'bxf'};{'bxf'};{'bxf'};{'bxf'};{'bxf'};{'bxf'};{'bxf'}]);
%traj_opt = traj_opt.addStateConstraint(c0,1);
%traj_opt = traj_opt_con.setName(cnstr_name)
%traj_opt = traj_opt.addStateConstraint(ConstantConstraint(xf),N);
traj_opt = traj_opt.addStateConstraint(Bx0,1);
traj_opt = traj_opt.addStateConstraint(Bxf,N);
%traj_opt = traj_opt.addInputConstraint(BoundingBoxConstraint(1,5),Z);
%traj_opt.addStateConstraint(Rolling_constraint,{[1, 2,3,4,5,6,7,8,9,10,11,11,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,45,46,47,48,49,50]});
%traj_opt.addStateConstraint(Rolling_constraint,{[2,3,4,5,6,7,8,9,10,]});
%traj_opt = traj_opt.addStateConstraint(Rolling_constraint,Z,[7;10]);
%traj_opt = traj_opt.addStateConstraint(input_constraint,Z,[3;4]);
traj_opt = traj_opt.addStateConstraint(height_constraint,Z,[2]);
%traj_opt = traj_opt.addStateConstraint(pitch_constraint,Z,[3]);
traj_opt =traj_opt.addTrajectoryDisplayFunction(@(t,x,u)plotTraj(t,x,u,[1 2]));
%traj_opt =traj_opt.addTrajectoryDisplayFunction(@(t,x,u)plotDircolTraj(t,x,u,[4 8]));
% traj_opt = traj_opt.setCheckGrad(true);
%snprint('snoptzz2.out');
%snsummary('summaryz2.out')

traj_opt = traj_opt.setSolver('snopt');
 traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',100000000);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',2000000000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',500000000);
 traj_opt = traj_opt.setSolverOptions('snopt','MajorOptimalityTolerance',1e-1);
  traj_opt = traj_opt.setSolverOptions('snopt','MajorFeasibilityTolerance',1e-4);
 traj_opt = traj_opt.setSolverOptions('snopt','MinorFeasibilityTolerance',1e-4);
  traj_opt = traj_opt.setSolverOptions('snopt',' LinesearchTolerance',0.9);
    traj_opt = traj_opt.setSolverOptions('snopt',' SuperbasicsLimit',3000);
 %traj_opt = traj_opt.setSolverOptions('snopt','sense ','Feasible point');
 traj_opt = traj_opt.setConstraintErrTol(1e-1);


tic 
[xtraj,utraj,ltraj,ljltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);
toc
%%
function [f, df] = running_cost_fun(h,x,u)
    %h=-10;
%     q=ones(1,8);
%     q(1)=10;
%     Q=diag(q);
%     xnom=[0.5*x(3);x(2);x(4);u(1);0.5*x(8);x(6);x(7);x(8)];
%     xbar=x-xnom;
%R=eye(3);
 f = h*u'*u;
 df = [u'*u zeros(1,12) 2*h*u'];
% f = u'*R*u;%+xbar'*Q*xbar;
 %df = [zeros(3,1) zeros(3,12) 2*R*u];
  %df = [u'*u 2*xbar'*Q  2*h*u'];
   %f = xbar'*Q*xbar;
  %df = [zeros(1,1) 2*xbar'*Q  0];
end
%%
function [h,dh] = final_cost_fun(t,x)
  q=10*ones(1,8);
  q(1)=10;
  Qf=diag(q);
  xd=[5;0.5;x(3);x(4);0.5*x(7);x(6);x(7);x(8)];
  xerr=x-xd;
  h =  xerr'*Qf*xerr;
  dh = [0,  2*xerr'*Qf];
end
%%
function plotTraj(t,x,u,plotdims)
    tt=[0;cumsum(t)];
    ttx=tt';
  figure(27);
  h1=plot(x(plotdims(1),:),x(plotdims(2),:),'r.-');
  title('$X$ vs $Z$','interpreter','latex');
  xlabel('$x$','interpreter','latex','FontSize',20);
  ylabel('$z$','interpreter','latex','FontSize',20);
  drawnow;
  figure(28);
  h2=plot(ttx(1,:),x(1,:),'g.-');
  title('$X$ evolution','interpreter','latex')
  xlabel('$t$','interpreter','latex','FontSize',20);
  ylabel('$x$','interpreter','latex','FontSize',20);
  drawnow;
  figure (29);
  h3=plot(ttx(1,:),u(:,:));
   title('$U$ evolution','interpreter','latex')
  xlabel('$t$','interpreter','latex','FontSize',20);
  ylabel('$u$','interpreter','latex','FontSize',20);
  legend('wheel','arm1','arm2')
  drawnow;
   figure(30);
  h4=plot(ttx(1,:),x(3,:),'g.-');
  title('$pitch$ evolution','interpreter','latex')
  xlabel('$t$','interpreter','latex','FontSize',20);
  ylabel('$pitch$','interpreter','latex','FontSize',20);
  drawnow;
     figure(31);
  h5=plot(ttx(1,:),x(4,:),'g.-');
  title('$theta$ evolution','interpreter','latex')
  xlabel('$t$','interpreter','latex','FontSize',20);
  ylabel('$\theta$','interpreter','latex','FontSize',20);
  drawnow;
  figure(32);
  h6=plot(ttx(1,:),x(5,:),'.-');
  title('$theta$ Arm 1 evolution','interpreter','latex')
  xlabel('$t$','interpreter','latex','FontSize',20);
  ylabel('$\theta$','interpreter','latex','FontSize',20);
  drawnow;
  figure(33);
   h7=plot(ttx(1,:),x(6,:),'.-');
  title('$theta$ Arm 2 evolution','interpreter','latex')
  xlabel('$t$','interpreter','latex','FontSize',20);
  ylabel('$\theta$','interpreter','latex','FontSize',20);
  drawnow;
  %delete(h);
end

%r = TimeSteppingRigidBodyManipulator(p,.001);
v = p.constructVisualizer;
v.axis = [-0.5 15 -0.5 4];

v.display_dt = .05;

v.playback(xtraj, struct('slider', true));
%playbackAVI(v,xtraj,'no height const');
end

