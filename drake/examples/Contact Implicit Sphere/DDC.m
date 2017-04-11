options.floating = true;
%plant = PlanarRigidBodyManipulator('TIWP_multicontact.urdf',options);
plant = PlanarRigidBodyManipulator('TWIPheavy.urdf',options);


%visualizer
v = constructVisualizer(plant);
v.axis = [-5 5 -5 5];


q=[0;%x
   0;%y
   0;%theta
   pi/4; % arm1
   pi/4; %arm2 
   0]; %leg
   
qd=zeros(6,1); %velocities

[H] = plant.manipulatorDynamics(q,qd);

v.draw(0,q);

Hbb=H(1:3,1:3)
Hbq=H(1:3,4:6)
Hqq=H(4:6,4:6)

Z=Hbq'-Hqq*(Hbq\Hbb);
Z_squared=Z'*Z;
Z2D=Z(:,1:2);
Z2D_squared=Z2D'*Z2D;
 C=[0; 0; 0];
 C2D=[0; 0];

% figure (3)
% Ellipse_plot(Z,C)
% figure (4)
% Ellipse_plot(Z_squared,C)
% figure (5)
% Ellipse_plot(Z2D_squared,C)

nsphere=50;

%Method 1 plot
[Xs, Ys, Zs]=sphere(nsphere);
Torque_sphere=[Xs(:),Ys(:),Zs(:)]';
Effect=Z*Torque_sphere;
EffectX=reshape(Effect(1,:),[nsphere nsphere]+1);
EffectY=reshape(Effect(2,:),[nsphere nsphere]+1);
figure (2)
hold on 
grid on
axis equal
plot(EffectX,EffectY);

%%%INverse Z plot
Z_inv=inv(Z);
Z_inv2D=Z_inv(1:2,:)
[U2 S2 V2]=svd(Z_inv2D)
Effect_inv=Z_inv*Torque_sphere;
EffectX_inv=reshape(Effect_inv(1,:),[nsphere nsphere]+1);
EffectY_inv=reshape(Effect_inv(2,:),[nsphere nsphere]+1);
EffectZ_inv=reshape(Effect_inv(3,:),[nsphere nsphere]+1);
figure (1)
%axis equal
hold on 
grid on 
%Ellipse_plot(Z2D_squared,C2D);
Ellipse_plot(Z_squared,C);
plot(Xs, Ys);
P1=Z_inv2D*[1;1;1]
plot(P1(1),P1(2),'*r')
%P=plot(EffectX_inv,EffectY_inv,'-');
%S=surf(EffectX_inv,EffectY_inv,EffectZ_inv);
%c=p.Color;
%p.color='red';
%p.LineWidth='1';

% SVD Plot 

[U S V]=svd(Z_inv)
Vx=U2(:,1);
Vy=U2(:,2);
sigmax=S2(1,1);
sigmay=S2(2,2);


%umaxout=A*Vmax;
plot([0 sigmax*Vx(1)],[0 sigmax*Vx(2)],'--r','LineWidth', 2) %this should line up the ellipse's major axis
plot([0 sigmay*Vy(1)],[0 sigmay*Vy(2)],'--r','LineWidth', 2) %this should line up the ellipse's major axis
%plot3([0 sigma1*Vmax(1)],[0 sigma1*Vmax(2)],[0 sigma1*Vmax(3)],'--r','LineWidth', 2) %this should line up the ellipse's major axis

figure 
axis equal
Ellipse_plot(Z2D_squared,C2D);
plot([0 sigmax*Vx(1)],[0 sigmax*Vx(2)],'--r','LineWidth', 2) %this should line up the ellipse's major axis
plot([0 sigmay*Vy(1)],[0 sigmay*Vy(2)],'--r','LineWidth', 2) %this should line up the ellipse's major axis




