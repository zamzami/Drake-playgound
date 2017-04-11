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
 %f = u'*R*u;%+xbar'*Q*xbar;
 %df = [zeros(3,1) zeros(3,12) 2*R*u];
 %df = [u'*u 2*xbar'*Q  2*h*u'];
 %f = xbar'*Q*xbar;
 %df = [zeros(1,1) 2*xbar'*Q  0];
end