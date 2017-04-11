N=4;
%if (nargin<1) N = ceil(10*rand); end
sys = PlanarNLinkCustom(N);
%x=sys.simulate([0 5],randn(2*N,1));
v=sys.constructVisualizer();
v.axis=[-.5 .5 -2.2 2.2]+(N-1)*[-1 1 -1.2 1.2];
[utraj,xtraj] = swingUpTrajectoryOpt(sys);
v.playback(xtraj);

t = getBreaks(xtraj);
xtraj_data = xtraj.eval(t); 
utraj_data = utraj.eval(t); 
%[T,U]=sys.energy(xtraj_data);

power1=utraj_data(1,:).*utraj_data(1,:);
power2=utraj_data(2,:).*utraj_data(2,:);
powertot=cumsum(power1+power2);

% figure (2)
% hold on
% plot(t,T,'MarkerSize',30,'-.',t,U,'MarkerSize',30)
% legend('T','U')

figure(6)
plot(t,power1,'o-.',t,power2,'o-','linewidth',2)
legend('power1','power2')
hold on
%figure (3)
%plot(t,Z)
%hold on 

figure (4)
hold on
plot(t,xtraj_data(2,:),'o-.',t,xtraj_data(3,:),'o-','linewidth',2)
legend('q2','q3') 

figure (5)
plot(t,powertot)
legend('Cumlative power','linewidth',2)
hold on