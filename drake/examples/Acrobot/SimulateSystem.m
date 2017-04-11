function [lastX,xtraj,utraj]=SimulateSystem(u,dt,N,xinit)
% runs forward dynamics for a pendulum system and animates (optionally) 
%dt: total simulation time 
%N: numeber of discretization points along the trajectory

if (nargin<1) u=5; end
if (nargin<2) dt=1; end 
if (nargin<3) N=10; end 
if (nargin<4) xinit=[0;0;0;0]; end


visualize= true;

%initialize plant
plant = AcrobotPlant;

%initialize simulation 
t= linspace(0,dt,N);
utraj=PPTrajectory(zoh(t,repmat(u,1,N)));
utraj = utraj.setOutputFrame(plant.getInputFrame);
sys = cascade(utraj,plant);

%simulate 
xtraj=simulate(sys,utraj.tspan,xinit);

%Get data 

t = getBreaks(xtraj);
lastX = xtraj.eval(length(t));


if visualize
pv = AcrobotVisualizer(plant);
pv.playback(xtraj);
figure 
plot(ppval(utraj.pp.breaks,utraj.pp)');
figure 
fnplt(xtraj);
end 

end