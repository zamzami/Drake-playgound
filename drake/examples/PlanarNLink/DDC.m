function [Z,Zinv,C,G,Hbq,Cbar,Cstar,Zstar,Torqueconttemp,Ccontribtemp]=DDC(plant,xtraj_data,utraj_data,t)

for i=1:length(xtraj_data(1,:))
[H(:,:,i),C(:,:,i)]=plant.manipulatorDynamics(xtraj_data(1:3,i),xtraj_data(4:6,i));
[~,G(:,:,i)]=plant.manipulatorDynamics(xtraj_data(1:3,i),zeros(3,1));

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
qddot_totaltemp(:,:,i)=Torqueconttemp(:,:,i)+Ccontribtemp(:,:,i);
end 
qddot_total=reshape(qddot_totaltemp,[1 21]);
Torquecont=reshape(Torqueconttemp,[1 21]);
Ccontrib=reshape(Ccontribtemp,[1 21]);
figure 
hold on 
plot(t,Torquecont,'o-.',t,Ccontrib,'o-.',t,qddot_total,'o-','linewidth',2)
l=legend('$\tilde{\tau}$','$\tilde{C}$','Total''$\ddot{q}_{p}$');
set(l,'Interpreter','latex')
title('Contribution Breakdown to the Acceleration of the Unactuated Joint','FontSize',15);
 xlabel('$Time (s)$','interpreter','latex','FontSize',20);
 ylabel('$\ddot{q}_{p} \: (rad/{{s}^2})$','interpreter','latex','FontSize',20);

end 

