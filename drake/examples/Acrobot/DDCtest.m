    l1 = 1; l2 = 2;  
    m1 = 1; m2 = 1;  
    g = 9.81;
    b1=.1;  b2=.1;
%    b1=0; b2=0;
    lc1 = .5; lc2 = 1; 
    Ic1 = .083;  Ic2 = .33;
    

      I1 = Ic1 + m1*lc1^2; I2 = Ic2 + m2*lc2^2;
      m2l1lc2 = m2*l1*lc2;  % occurs often!
      
     %q1=0;q2=0;
     % q=[q1;q2];
      %c =@(q)cos(q(1:2,:));  s = sin(q(1:2,:));  s12 = sin(q(1,:)+q(2,:));
      h12 =@(q) I2 + m2l1lc2*cos(q);
      %H =[ I1 + I2 + m2*l1^2 + 2*m2l1lc2*cos(q), h12; h12, I2 ];
      h11=@(q)I1 + I2 + m2*l1^2 + 2*m2l1lc2*cos(q);
      h12_inv=@(q) 1/(I2 + m2l1lc2*cos(q));
      h22=I2;
      
      
      Z=@(q)h12(q)-h22*h12_inv(q)*h11(q)
%       
%       kinsol = doKinematics(obj, q, v, options);
%      [J, v_indices] = geometricJacobian(manipulator, kinsol, world, i, world);
%      
%       kinsol = doKinematics(obj.r.getManipulator,target_pose(1:obj.r.getNumPositions));
%       pt = forwardKin(obj.r.getManipulator,kinsol,6,[0.3,0,0.0].');
% 
%      world = 1;
%      
  
