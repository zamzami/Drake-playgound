Torquecont=reshape(Torqueconttemp,[2 21]);
Ccontrib=reshape(Ccontribtemp,[2 21]);
figure 
hold on 
plot(t,Torquecont,t,Ccontrib)
legend('Tau_bar','C_bar')