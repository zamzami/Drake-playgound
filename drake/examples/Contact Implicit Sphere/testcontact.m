
%construct robot model
options.terrain = RigidBodyFlatTerrain();
options.ignore_self_collisions = true;
options.floating = true;
%options.floating = 'quat';
options.ignore_friction = true;
options.replace_cylinders_with_capsules = false;
options.use_bullet=false;
%plant = RigidBodyManipulator('wheelbot.urdf',options);
%plant = RigidBodyManipulator('TWIP.urdf',options);
plant = RigidBodyManipulator('ball.urdf',options);
Ball=plant.body(3);
Ball.coulomb_friction=0.5;
%plant = PlanarRigidBodyManipulator('lightmm.urdf',options);
%robot = robot.addRobotFromURDF('wheel.urdf', [0;0;1], [-pi/2;0;0]);
%robot = robot.addRobotFromURDF('mm.urdf',options);
%robot = compile(robot);

%resolve initial conditioncoulomb_friction
%xstar_hand = robot.getInitialState();
%xstar_hand = robot.resolveConstraints(xstar_hand);
f_base_position=[0;0;0.7];
f_base_orientation=[0;0;0];
configuration_vector=[0]; %wheel,arm1,arm2
dconfiguration_vector=zeros(7,1);
%run simulation
N=100; tf=5;
step=tf/(N-1);
%ts = 0:0.01:12;
ts = 0:step:tf;
%x0 = [0;0;.3;rpy2quat(zeros(3,1));zeros(3,1);0;0;0;0;0];
x0 = [f_base_position;f_base_orientation;configuration_vector;dconfiguration_vector];
%x0 = [0;.3;zeros(6,1)];
%N=10; tf=.5;
robot = TimeSteppingRigidBodyManipulator(plant, 0.001);

q=sym('q',[7,1]); q=sym(q,'real');
qdot=sym('qd',[7,1]); qdot=sym(qdot,'real');
[H,C,B]=manipulatorDynamics(plant,q,qdot)

%resolve initial condition
%xstar = robot.getInitialState();
%xstar = robot.resolveConstraints(x0);
xtraj_ts = simulate(robot,[0 tf],x0);

t = 0:step:tf;
wheel_torque_traj=PPTrajectory(foh(linspace(0,tf,N),20*linspace(0,tf,N)));
% a1_torque_traj=PPTrajectory(foh(linspace(0,tf,N),0*linspace(0,tf,N)));
% %a2_torque_traj=PPTrajectory(foh(linspace(0,1,N),10*linspace(0,1,N)),foh(linspace(0,1,N),-10*linspace(0,1,N)));
% a2_zero_traj=PPTrajectory(foh(linspace(0,2,N),0*linspace(0,2,N)));
% a2_acc_traj=PPTrajectory(foh(linspace(0,1,N),10*linspace(0,1,N)));
% a2_decc_traj=PPTrajectory(foh(linspace(0,1,N),-10*linspace(0,1,N)));
% a2_composed_traj=PPTrajectory(foh(linspace(0,tf,N),[0*linspace(0,4,4*N/10) -50*linspace(0,2,2*N/10) -50*linspace(0,2,2*N/10) -100*linspace(0,1,N/10) 100*linspace(0,1,N/10)]));


%utraj = setOutputFrame(PPTrajectory(spline(ts,cos(2*ts))),getInputFrame(robot));
utraj = setOutputFrame(wheel_torque_traj,getInputFrame(robot));
% utraj = setOutputFrame([wheel_torque_traj;a1_torque_traj;a2_composed_traj],getInputFrame(robot));
 ytraj = simulate(cascade(utraj,robot),[0 tf],x0);


%playback
v = robot.constructVisualizer;
%v.enableLCMGLInertiaEllipsoids;
%v.axis = [-0.5 25 -0.5 0.5];
%v.use_collision_geometry=true;  
%v.playback(xtraj_ts, struct('slider', true));
v.playback(ytraj, struct('slider', true));
%v.playbackSWF(ytraj,'roll128ptsv20')

% %Plot function 
% function plotTraj(t,x,u,plotdims)
%     tt=[0;cumsum(t)];
%     ttx=tt';
  % figure(27);
 %  h1=plot(x(plotdims(1),:),x(plotdims(2),:),'r.-');
%   title('$X$ vs $Z$','interpreter','latex');
%   xlabel('$x$','interpreter','latex','FontSize',20);
%   ylabel('$z$','interpreter','latex','FontSize',20);
%   drawnow;
% end 


