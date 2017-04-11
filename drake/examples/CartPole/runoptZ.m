function  [xtraj,utraj,z,F,info]=runoptZ()
%NOTEST
sys = CartPolePlant();

x0 = zeros(4,1); tf0 = 4;
xf=[0;pi;0;0];
N=21;
%utraj0 = PPTrajectory(zoh(linspace(0,tf0,51),randsn(1,51)));
%utraj0 = PPTrajectory(zoh(linspace(0,tf0,N),randn(1,N)));
% options = struct('maxDT',0.05,'Tmin',2,'Tmax',6,'xf',[0;pi;0;0]);%,'bGradTest',true);
% [obj.xtraj,obj.utraj,info] = DirtranTrajectoryOptimization(sys,@cost,@finalcost,x0,utraj0,options);
% if (info~=1) error('dirtran failed to find a trajectory'); end

prog = DirtranTrajectoryOptimization(sys,N,[2 6]);
prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@finalcost);
prog = prog.addStateConstraint(ConstantConstraint(x0),1);
prog = prog.addStateConstraint(ConstantConstraint(xf),N);

traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
traj_init.u = PPTrajectory(zoh(linspace(0,tf0,N),randn(1,N)));
u = PPTrajectory(zoh(linspace(0,tf0,N),randn(1,N)));


for attempts=1:10
        tic
        [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,u);
        toc
        if info==1, break; end
end
end 

   





function [g,dg] = cost(t,x,u)
  xd = repmat([0;pi;0;0],1,size(x,2));
  x(2,:) = mod(x(2,:),2*pi);
  xerr = x - repmat(xd,1,size(x,2));
  Q = diag([10,10,1,1]);  R=1;
  
  g = sum((Q*xerr).*xerr,1) + (R*u).*u;

  if (nargout>1)
    dgdt = 0;
    dgdx = 2*xerr'*Q;
    dgdu = 2*u'*R;
    dg{1} = [dgdt,dgdx,dgdu];
  end
end
  
function [h,dh] = finalcost(t,x)
  xd = repmat([0;pi;0;0],1,size(x,2));
  x(2,:) = mod(x(2,:),2*pi);
  xerr = x - repmat(xd,1,size(x,2));
  Qf = diag([10,10,1,1]);
  h = sum((Qf*xerr).*xerr,1);

  if (nargout>1)
    dhdt = 0;
    dhdx = 2*xerr'*Qf;
    dh{1} = [dhdt,dhdx];
  end
end
