% Run MMdynamics First 

%Z=load('AnalyticalInertia_legged_MM','-mat','Z')
%Floating base paramters 
Base.m0=0.1;
Base.l0=0;
Base.d=0.1;
%Base.I0=1/12*Base.m0*(Base.d)^2;
Base.I0=0.0004; % URDF Values
%Base.I0=2/5*Base.m0*(Base.d)^2;

%Arm 1 link paramters
Arm1.m1=1;
Arm1.l1=1;
Arm1.c1=0.5;
Arm1.radius=0.1;
%Arm1.I1=1/12*Arm1.m1*(3*Arm1.radius^2 +Arm1.l1^2);
Arm2.I1=0.085833;
%Arm link 2 paramters 
Arm2.m2=1;
Arm2.l2=1;
Arm2.c2=0.5;
Arm2.radius=0.1;
%Arm2.I2=1/12*Arm2.m2*(3*Arm2.radius^2+Arm2.l2^2);
Arm2.I2=0.085833;

%leg parameters
leg.mleg=0.1;
leg.cleg=0.25;
leg.lleg=0.5;
%leg.Ileg=1/12*Arm2.m2*(Arm2.l2)^2;
leg.Ileg=0.0023;

%General coordinates
Q.phi=0;
Q.q1=pi/2;
Q.q2=0;
Q.alpha=0;

ParamValues=struct('m0',Base.m0,'l0',Base.l0,'I0',Base.I0,...
                   'm1',Arm1.m1,'l1',Arm1.l1,'c1',Arm1.c1,'I1',Arm1.I1,...
                    'm2',Arm2.m2,'l2',Arm2.l2,'c2',Arm2.c2,'I2',Arm2.I2,...
                    'mleg',leg.mleg,'cleg',leg.cleg,'Ileg',leg.Ileg,...
                    'phi',Q.phi,'q1',Q.q1,'q2',Q.q2,'alpha',Q.alpha);
                
H_numerical=eval(subs(M,ParamValues))
Z_numerical=subs(Z,ParamValues)
Z_PI=eval((Z_numerical.')*Z_numerical)

Z_inv=eval(inv(Z_numerical))
%Z_inv_pos=eval(Z_inv(1:2,:))

%xbound=linspace(-1,1,100);
%ybound=linspace(-1,1,100);
%zbound=linspace(-1,1,100);

%[xb,yb,zb]=ndgrid(xbound,ybound,zbound);

% figure(3)
% plot3(xb,yb,zb,'.')
Z_numerical_eval=eval(Z_numerical)

%Effect=Z_inv_pos.*Torque_sphere;
Effect=Z_numerical_eval*Torque_sphere;
EffectX=reshape(Effect(1,:),[100 100]+1);
EffectY=reshape(Effect(2,:),[100 100]+1);
figure (2)
hold on 
grid on
axis equal
%plot(EffectX,EffectY);
%%%INverse Z plot
Effect_inv=Z_inv*Torque_sphere;
EffectX_inv=reshape(Effect_inv(1,:),[100 100]+1);
EffectY_inv=reshape(Effect_inv(2,:),[100 100]+1);
%plot(EffectX_inv,EffectY_inv);
Effect_inv_p=Z_inv*[0.9;0;0];
plot(Effect_inv_p(1),Effect_inv_p(2),'*');
%%% W SVD plot %%
C=[0; 0; 0];
%Ellipse_plot(Z_PI,C)
%subs(Z,[m0, m1, m2, mleg,c1, c2, cleg,l0,l1,I1,I2,Ileg,I0 phi, q1, q2,alpha],[1, 1, 1,0.1, 1, 1, 1,1,2,0.1,0.1,0.01,0.2,0,0,0,0.8*pi/2])