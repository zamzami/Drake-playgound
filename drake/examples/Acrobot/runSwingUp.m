function [p,v,xtraj,utraj,T,U,power]=runSwingUp()
%% runs trajectory optimization and animates open-loop playback

p = AcrobotPlant;
v = AcrobotVisualizer(p);
[utraj,xtraj] = swingUpTrajectory(p);
%      sys = cascade(utraj,p);
%      xtraj=simulate(sys,utraj.tspan,zeros(4,1));
v.playback(xtraj);

t = getBreaks(xtraj);
xtraj_data = xtraj.eval(t); 
utraj_data = utraj.eval(t); 
[T,U]=p.energy(xtraj_data);
%[~,~,~,Z]=p.manipulatorDynamics(xtraj_data(1:2,:),xtraj_data(3:4,:));
power=utraj_data.*utraj_data;
powertot=cumsum(power);
figure (2)
plot(t,T,'MarkerSize',30)
hold on 
plot(t,U,'MarkerSize',30)
hold on 
figure(6)
plot(t,power)
hold on
%figure (3)
%plot(t,Z)
%hold on 
figure (4)
plot(t,xtraj_data(2,:))
hold on

figure (5)
plot(t,powertot)
hold on
end
