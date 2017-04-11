function [Z,Zinv,C,G,Hbq,Cbar,Cstar,Zstar,Torqueconttemp,Ccontribtemp,Xbias,ZZ,ZZ2,ZZ2D,Xbias2D]=ODDC(plant,xtraj_data,utraj_data,t)

for i=1:length(xtraj_data(1,:))
 kinsol = plant.doKinematics(xtraj_data(1:3,i),xtraj_data(4:6,i));
 Jtemp= geometricJacobian(plant, kinsol, 1, 4,1);
 J(:,:,i)=[Jtemp(2,:);Jtemp(4,:);Jtemp(6,:)];
 J2D(:,:,i)=J(2:3,:,i);
 J22D(:,:,i)=J(2:3,2:3,i);
Jdottemp= geometricJacobianDotTimesV(plant, kinsol, 1, 4,1);
Jdot_times_v(:,:,i)=[Jdottemp(2,:);Jdottemp(4,:);Jdottemp(6,:)];


[H(:,:,i),C(:,:,i)]=plant.manipulatorDynamics(xtraj_data(1:3,i),xtraj_data(4:6,i));
[~,G(:,:,i)]=plant.manipulatorDynamics(xtraj_data(1:3,i),zeros(3,1));

Hinv(:,:,i)=inv(H(:,:,i));

JMC(:,:,i)=J(:,:,i)*Hinv(:,:,i)*C(:,:,i);

L=[10 0;0 10];
L_2=inv(L)^2;

Xbias(:,:,i)=Jdot_times_v(:,:,i)-JMC(:,:,i);
Xbias2D(:,:,i)=Xbias(2:3,:,i);

H2D(:,:,i)=H(2:3,2:3,i);
Q2D(:,:,i)=H2D(:,:,i)*L_2*H2D(:,:,i);
%Qinv2D(:,:,i)=inv(Q2D(:,:,i));
Jinv2D(:,:,i)=pinv(J2D(:,:,i));
Jinv22D(:,:,i)=pinv(J22D(:,:,i));
ZZ2(:,:,i)=Jinv22D(:,:,i)'*Q2D(:,:,i)*Jinv22D(:,:,i);


Q(:,:,i)=H(:,:,i)*H(:,:,i);
%Qinv(:,:,i)=inv(Q(:,:,i));
Jinv(:,:,i)=pinv(J(:,:,i));
ZZ(:,:,i)=Jinv(:,:,i)'*Q(:,:,i)*Jinv(:,:,i);

Jinv2D(:,:,i)=pinv(J2D(:,:,i));

ZZ2D(:,:,i)=Jinv2D(:,:,i)'*Q(:,:,i)*Jinv2D(:,:,i);


%WeightedJPseudo(:,:,i)=Qinv(:,:,i)*J(:,:,i)'*inv((J(:,:,i)*Qinv(:,:,i)*J(:,:,i)'));

Hbq(:,:,i)=H(1,2:3,i);
Hqq(:,:,i)=H(2:3,2:3,i);
Hbqinv(:,:,i)=pinv(Hbq(:,:,i));
Hbb(:,:,i)=H(1,1,i);

Z(:,:,i)=Hbq(:,:,i)'-Hqq(:,:,i)*Hbqinv(:,:,i)*Hbb(:,:,i);
Zinv(:,:,i)=pinv(Z(:,:,i));
Cq(:,:,i)=C(2:3,:,i);
Cb(:,:,i)=C(1,:,i);
Cbar(:,:,i)=Cq(:,:,i)-Hqq(:,:,i)*Hbqinv(:,:,i)*Cb(:,:,i);
Cstar(:,:,i)=Zinv(:,:,i)*Cbar(:,:,i);
Zstar(:,:,i)=Zinv(:,:,i)'*Zinv(:,:,i);

Torqueconttemp(:,:,i)=Zinv(:,:,i)*utraj_data(:,i);
Ccontribtemp(:,:,i)=-Cstar(:,:,i);
end 
Torquecont=reshape(Torqueconttemp,[1 21]);
Ccontrib=reshape(Ccontribtemp,[1 21]);
figure 
hold on 
plot(t,Torquecont,'o-.',t,Ccontrib,'o-.','linewidth',2)
legend('Tau bar','C bar')

end 

