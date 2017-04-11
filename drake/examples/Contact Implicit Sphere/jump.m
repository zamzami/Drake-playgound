%rng(0)
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.use_bullet=false;
options.ignore_self_collisions = false;
options.replace_cylinders_with_capsules = false;
%options.floating = 'quat';
%w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
plant = PlanarRigidBodyManipulator('TWIP2D.urdf',options);
%plant = addRobotFromURDF(plant, 'obstacle.urdf',[1.5;-0.7;0]);
%plant = RigidBodyManipulator('ball.urdf',options);
%plant = RigidBodyManipulator('wheel.urdf',options);
%plant = RigidBodyManipulator('Capsule.urdf',options);
%warning(w);
%x0 = [0;0;1;0.05*randn(3,1);zeros(6,1)];

%xstar_hand = robot.getInitialState();
%xstar_hand = robot.resolveConstraints(xstar_hand);


%x0 = [0;0;.11;zeros(3,1);zeros(3,1);0;0;0];


%plant_ts = TimeSteppingRigidBodyManipulator(plant,tf/(N-1));
%w = warning('off','Drake:TimeSteppingRigidBodyManipulator:ResolvingLCP');
%xtraj_ts = simulate(plant_ts,[0 tf],x0);
%x0 = xtraj_ts.eval(0);
%warning(w);

   %v = constructVisualizer(plant_ts);
   %v.playback(xtraj_ts);
 
 % Timing parameters   
  N=10; tf=0.5; 
  t_init = linspace(0,tf,N);
  N2 = floor(0.5*N);
  u0=[0;0;0];

%hip=[1.4;1.45;1.55;1.4;1.3;10.96;0.6;-0.087]
%hip2=[1.3;1.35;1.4;1.45;1.5;1.75;2;2.2;2.5;2.9]
%Knee=[0.5;0.51;0.54;0.6;0.7;0.8;0.9;1.1;1.25;1.5]

%hip=[1.8416; 1.7916 ;1.7416; 1.6916; 1.6416; 1.3916; 1.1416 ;0.9416;
%0.6416;0.2416]%arm 2

%Knee=[   -1.0708;-1.0608;-1.0308;-0.9708;-0.8708;-0.7708;-0.6708;-0.4708;-0.3208;-0.0708] %arm1
%leg=[ -0.7708; -0.7708; -0.7908; -0.7808; -0.7708; -0.7608; -0.6708; -0.5708; -0.3708;-0.0708];

%Initial state   
f_base_position=[0;0.8];
f_base_orientation=zeros(1,1);
configuration_vector=[0.7;-1.071;1.8416]; %wheel,arm1,arm2
base_velocity=[0;0;0];
dconfiguration_vector=zeros(3,1);
x0 = [f_base_position;f_base_orientation;configuration_vector;base_velocity;dconfiguration_vector];


%Mid contact phase  @t=0.2
x11_f_base_position=[0;0.8];
x11_f_base_orientation=zeros(1,1);
x11_configuration_vector=[0.7;-0.7708;1.3916]; %wheel,arm1,arm2
x11_base_velocity=[0;1;0];
x11_dconfiguration_vector=[0;-5;5];
x1 = [x11_f_base_position;x11_f_base_orientation;x11_configuration_vector;x11_base_velocity;x11_dconfiguration_vector];

%take-off phase  @t=0.4
x12_f_base_position=[0;0.8];
x12_f_base_orientation=zeros(1,1);
x12_configuration_vector=[0.7;-0.0708;0.2416]; %wheel,arm1,arm2
x12_base_velocity=[0;0;0];
x12_dconfiguration_vector=[0;-5;5];
x2 = [x12_f_base_position;x12_f_base_orientation;x12_configuration_vector;x12_base_velocity;x12_dconfiguration_vector];

%Final Jump apex  @t=0.6 
x3_f_base_position=[0;1.2];
x3_f_base_orientation=zeros(1,1);
x3_configuration_vector=[0.07;-0.0708;0.2416]; %wheel,arm1,arm2
x13_base_velocity=[0;0;0];
x3_dconfiguration_vector=zeros(3,1);
x3 = [x3_f_base_position;x3_f_base_orientation;x3_configuration_vector;x13_base_velocity;x3_dconfiguration_vector];

%touch down   @t=0.8 
xf_f_base_position=[0;1];
xf_f_base_orientation=zeros(1,1);
xf_configuration_vector=[0.07;-0.0708;0.2416]; %wheel,arm1,arm2
xf_base_velocity=[0;0;0];
xf_dconfiguration_vector=zeros(3,1);
xf = [xf_f_base_position;xf_f_base_orientation;xf_configuration_vector;xf_base_velocity;xf_dconfiguration_vector];



    Njump = floor(0.1*N);
    C_initial=[1; 1; 1; 0;1; 1; 1; 0];
    C_takeoff=[0; 0; 0; 0;0; 0; 0; 0];
  %traj_init.x = PPTrajectory(foh([0,tf],[x0,x0]));
   traj_init.l = PPTrajectory(foh(t_init,[repmat(C_initial,1,6),repmat(C_takeoff,1,4)]));
   
  traj_init.x = PPTrajectory(foh(t_init,[linspacevec(x0,x1,3),linspacevec(x1,x2,3),linspacevec(x2,x3,4)]));
  traj_init.x = traj_init.x.setOutputFrame(plant.getStateFrame);
  traj_init.u = PPTrajectory(foh(t_init,[0*randn(1,N);0*randn(2,N)]));
  traj_init.u = traj_init.u.setOutputFrame(plant.getInputFrame);

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
    prog = prog.setSolver('snopt');
   prog = prog.setSolverOptions('snopt','MajorIterationsLimit',200);
   prog = prog.setSolverOptions('snopt','MinorIterationsLimit',20000);
   prog = prog.setSolverOptions('snopt','IterationsLimit',20000);
   prog = prog.setSolverOptions('snopt',' SuperbasicsLimit',3000);
  prog = prog.setSolverOptions('snopt','MajorFeasibilityTolerance',1e-4);
  prog = prog.setSolverOptions('snopt','MinorFeasibilityTolerance',1e-4);

  % prog = prog.setCheckGrad(true);
  
%    snprint('snopt.out');
%    snprint('snoptTWIP.out');
   snsummary('summaryzTWIP.out')
  
  % Boundary conditions constraint
  prog = addStateConstraint(prog,ConstantConstraint(x0),1);
  prog = addStateConstraint(prog,ConstantConstraint(x2),6);
  prog = addStateConstraint(prog,ConstantConstraint(x3),10);
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
  v.axis = [-2 2 -0.5 5];
  %v.display_dt=0.01;
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
legendmatrixT{4}='T_init1';
legendmatrixT{5}='T_init2';
legendmatrixT{6}='T_init3';

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
  

  


