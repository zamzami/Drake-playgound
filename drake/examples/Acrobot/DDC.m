function [Z,Zinv,C,G,Hbq,Cbar,Cstar,Zstar]=DDC(plant,xtraj_data,utraj_data,t)

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
end 
Torquecont=reshape(Torqueconttemp,[2 21]);
Ccontrib=reshape(Ccontribtemp,[2 21]);
figure 
hold on 
plot(t,Torquecont,t,Ccontrib)
legend('Tau_bar','C_bar')

end 

