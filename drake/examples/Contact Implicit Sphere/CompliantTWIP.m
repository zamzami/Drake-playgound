%rng(0)
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.use_bullet=false;
options.ignore_self_collisions = true;
options.replace_cylinders_with_capsules = false;
%options.floating = 'quat';
%w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
plant = PlanarRigidBodyManipulator('TWIPspring.urdf',options);
plant = addRobotFromURDF(plant, 'Box.urdf',[2;0]);

  v = constructVisualizer(plant);
  v.axis = [-0.5 5 -0.5 5];

%plant = RigidBodyManipulator('ball.urdf',options);
%plant = RigidBodyManipulator('wheel.urdf',options);
%plant = RigidBodyManipulator('Capsule.urdf',options);
%warning(w);
%x0 = [0;0;1;0.05*randn(3,1);zeros(6,1)];

%xstar_hand = robot.getInitialState();
%xstar_hand = robot.resolveConstraints(xstar_hand);


f_base_position=[0;1];
f_base_orientation=zeros(1,1);
configuration_vector=[0;0;0;0]; %wheel,arm1,arm2
dconfiguration_vector=zeros(7,1);
x0 = [f_base_position;f_base_orientation;configuration_vector;dconfiguration_vector];
%x0 = [0;0;.11;zeros(3,1);zeros(3,1);0;0;0];
N=20; tf=2;

%plant_ts = TimeSteppingRigidBodyManipulator(plant,tf/(N-1));
%w = warning('off','Drake:TimeSteppingRigidBodyManipulator:ResolvingLCP');
%xtraj_ts = simulate(plant_ts,[0 tf],x0);
%x0 = xtraj_ts.eval(0);
%warning(w);

   %v = constructVisualizer(plant_ts);
   %v.playback(xtraj_ts);
 
   
   
  t_init = linspace(0,tf,N);
  N2 = floor(0.5*N);
  
 % Initial state  
f_base_position=[0;1];
f_base_orientation=zeros(1,1);
configuration_vector=[0;0;0;0]; %wheel,arm1,arm2
dconfiguration_vector=zeros(7,1);
x0 = [f_base_position;f_base_orientation;configuration_vector;dconfiguration_vector];
  
% Final state  
xf_f_base_position=[0;2];
xf_f_base_orientation=zeros(1,1);
xf_configuration_vector=[0;0;0;0]; %wheel,arm1,arm2
xf_dconfiguration_vector=zeros(7,1);
xf = [xf_f_base_position;xf_f_base_orientation;xf_configuration_vector;xf_dconfiguration_vector];

% mid state
x1_f_base_position=[0;1.5];
x1_f_base_orientation=zeros(1,1);
x1_configuration_vector=[0;0;-pi/4;pi/4]; %wheel,arm1,arm2
x1_dconfiguration_vector=randn(7,1);
x1 = [x1_f_base_position;x1_f_base_orientation;x1_configuration_vector;x1_dconfiguration_vector];

% couple trajectory
  u0=[0;0;0];
  u1=[0;10;10];
  u2=[0;-10;-10];
  uf=[0;0;0];
  
  Njerk=floor(0.1*N);


  



    Njump = floor(0.1*N);
    C=[1; 1; 1; 1];
    C_mid=[0; 0; 0; 0];
  %traj_init.x = PPTrajectory(foh([0,tf],[x0,x0]));
   traj_init.l = PPTrajectory(foh(t_init,[repmat(C,1,Njump),repmat(C_mid,1,N-Njump)]));
   
  traj_init.x = PPTrajectory(foh(t_init,[linspacevec(x0,x1,N2),linspacevec(x1,xf,N-N2)]));
  %traj_init.x = traj_init.x.setOutputFrame(plant.getStateFrame);
  %traj_init.u = PPTrajectory(foh(t_init,[0*randn(1,N);randn(2,N)]));
  traj_init.u = PPTrajectory(foh(t_init,[linspacevec(u0,u1,Njerk),linspacevec(u1,u2,Njerk),linspacevec(u2,uf,N-2*Njerk)]));
  %traj_init.u = traj_init.u.setOutputFrame(plant.getInputFrame);

options = struct();
options.integration_method = ContactImplicitTrajectoryOptimization.MIXED;
options.use_bullet=false;


scale_sequence = [1;.001;0];

for i=1:length(scale_sequence)
  scale = scale_sequence(i);

  options.compl_slack = scale*.01;
  options.lincompl_slack = scale*.001;
  options.jlcompl_slack = scale*.01;
  options.lambda_mult = plant.getMass*9.81*tf/N;
  
  prog = ContactImplicitTrajectoryOptimization(plant,N,tf,options);
  prog = prog.setSolver('ipopt');
%  prog = prog.setSolverOptions('snopt','MajorIterationsLimit',200);
%  prog = prog.setSolverOptions('snopt','MinorIterationsLimit',20000);
%  prog = prog.setSolverOptions('snopt','IterationsLimit',20000);
%  prog = prog.setSolverOptions('snopt',' SuperbasicsLimit',3000);
%  prog = prog.setSolverOptions('snopt','MajorFeasibilityTolerance',1e-4);
%  prog = prog.setSolverOptions('snopt','MinorFeasibilityTolerance',1e-4);

  % prog = prog.setCheckGrad(true);
  
   snprint('snopt.out');
   snprint('snoptTWIP.out');
   snsummary('summaryzTWIP.out');
%   
  % Boundary conditions constraint
  prog = addStateConstraint(prog,ConstantConstraint(x0),1);
  %prog = addStateConstraint(prog,ConstantConstraint(xf),N);

 %  prog = addStateConstraint(prog,ConstantConstraint(0.5),N,1); % xf
%   prog = addStateConstraint(prog,ConstantConstraint(1),N,2); %yf
%   prog = addStateConstraint(prog,ConstantConstraint(0),N,3); % pitch
%   prog = addStateConstraint(prog,ConstantConstraint(0),N,4); % theta leg
%   prog = addStateConstraint(prog,ConstantConstraint(0),N,5); % theta arm1
%   prog = addStateConstraint(prog,ConstantConstraint(0),N,6); % theta arm2
%   
 %prog = prog.addRunningCost(@running_cost_fun);
 %prog = prog.addFinalCost(@final_cost_fun1);
   
  [xtraj,utraj,ltraj,~,z,F,info] = solveTraj(prog,tf,traj_init);
end
%utrajs = setOutputFrame(utraj,getInputFrame(plant_ts));
% utraj = setOutputFrame([wheel_torque_traj;a1_torque_traj;a2_composed_traj],getInputFrame(robot));
 %ytraj = simulate(cascade(utrajs,plant_ts),[0 tf],x0);
 
  %v.playback(ytraj);
  v = constructVisualizer(plant);
  v.playback(xtraj);
  v.axis = [-0.5 5 -0.5 5];
  v.playback(xtraj, struct('slider', true))
  %v.axis = [-0.5 25 -0.5 0.5];
%v.use_collision_geometry=true;  
%v.playback(xtraj_ts, struct('slider', true));
%v.playback(ytraj, struct('slider', true));
%v.playbackSWF(ytraj,'roll128ptsv20')

legendmatrix=cell(6,1);
legendmatrix{1}='TSx';
legendmatrix{2}='TSy';
legendmatrix{3}='TSz';
legendmatrix{4}='x';
legendmatrix{5}='y';
legendmatrix{6}='z';

legendmatrixT=cell(6,1);

legendmatrixT{1}='Topt1';
legendmatrixT{2}='Topt2';
legendmatrixT{3}='Topt3';
legendmatrixT{4}='Topt4';
legendmatrixT{5}='T_init1';
legendmatrixT{6}='T_init2';
legendmatrixT{7}='T_init3';
legendmatrixT{8}='T_init4';

legendmatrixl=cell(8,1);

legendmatrixl{1}='Topt1';
legendmatrixl{2}='Topt2';
legendmatrixl{3}='Topt3';
legendmatrixl{4}='Topt4';
legendmatrixl{5}='T_init1';
legendmatrixl{6}='T_init2';
legendmatrixl{7}='T_init3';
legendmatrixl{8}='T_init4';



% for k=1:length(Tsstring)
%  legendmatrix{k}=strcat('text',num2str(Tsstring(k)))
% end
% check if the two simulations did the same thing:
ts = getBreaks(utraj);
valuecheck(ts,getBreaks(xtraj));
xtraj_data = xtraj.eval(ts); 
%xtraj_ts_data = ytraj.eval(ts);

ltraj_data_init=traj_init.l.eval(ts);
ltraj_data = ltraj.eval(ts); 
utraj_data = utraj.eval(ts); 
utraj_data_init = traj_init.u.eval(ts); 
nq = plant.getNumPositions();
nv = plant.getNumVelocities();
%valuecheck(xtraj_data(1:nq,:),xtraj_ts_data(1:nq,:),position_tol); % is there a correct tolerance here?
%valuecheck(xtraj_data(nq+(1:nv),:),xtraj_ts_data(nq+(1:nv),:),velocity_tol); % is there a correct tolerance here?

%   figure (20);
%   p1=plot(ts,xtraj_ts_data(1:3,:),'.-', 'MarkerSize',25);
%   title('$Position$','interpreter','latex');
%   xlabel('$t$','interpreter','latex','FontSize',30);
%   ylabel('$z$','interpreter','latex','FontSize',30);
%   drawnow;
%   hold on 
%   p2=plot(ts,xtraj_data(1:3,:),'.-');
%   legend(legendmatrix)
%   legend([p1,p2],'Timestepping', 'trajopt');
%   
%     figure (21);
%   p1=plot(ts,xtraj_ts_data(4:nq,:),'.-', 'MarkerSize',25);
%   title('$Orientation$','interpreter','latex');
%   xlabel('$t$','interpreter','latex','FontSize',30);
%   ylabel('$z$','interpreter','latex','FontSize',30);
%   drawnow;
%   hold on 
%   p2=plot(ts,xtraj_data(4:nq,:),'.-');
%   legend(legendmatrix)
%   
     figure (22);
  p1=plot(ts,utraj_data(1:3,:),'.-', 'MarkerSize',25);
  title('$Torque$','interpreter','latex');
  xlabel('$t$','interpreter','latex','FontSize',30);
  ylabel('$torque$','interpreter','latex','FontSize',30);
  %drawnow;
  hold on 
  p2=plot(ts,utraj_data_init(1:3,:),'.-');
  legend(legendmatrixT)
  
       figure (23);
  p1=plot(ts,ltraj_data(1:4,:),'.-', 'MarkerSize',25);
  title('$Lambda$','interpreter','latex');
  xlabel('$t$','interpreter','latex','FontSize',30);
  ylabel('$lambda$','interpreter','latex','FontSize',30);
  %drawnow;
  hold on 
  p2=plot(ts,ltraj_data_init(1:4,:),'-');
  legend(legendmatrixl)
  
  
  %F12=arrayfun(@(x) x.force(12),lR,'unif',true)

  


