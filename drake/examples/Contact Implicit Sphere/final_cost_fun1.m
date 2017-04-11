function [h,dh] = final_cost_fun1(t,x)
  q=zeros(1,12);
  q(1)=2;
  q(2)=2;
  Qf=diag(q);
  xd=[1.5;0.5;x(3);x(4);x(5);x(6);x(7);x(8);x(9);x(10);x(11);x(12)];
  xerr=x-xd;
  h = t + xerr'*Qf*xerr;
  dh = [1,  2*xerr'*Qf];
end