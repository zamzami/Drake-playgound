% figure 
% plot(t,utraj_data(1,:),'o-.',t,utraj_data(2,:),'o-','linewidth',2)
% legend('Joint 2','Joint 3')
% title('Joint Effort Profile');
% %title('$Joint\:Effort\: Profile$','interpreter','latex','FontSize',20);
% xlabel('$Time (s)$','interpreter','latex','FontSize',20);
% ylabel('$Torque  (N.m)$','interpreter','latex','FontSize',20);

% figure 
% plot(t,xtraj_data(1,:),'o-.',t,xtraj_data(2,:),'o-',t,xtraj_data(3,:),'o-','linewidth',2)
% legend('Passive Joint 1','Active Joint 2','Active Joint 3')
% title('Joint Position Profile');
% %title('$Joint\:Effort\: Profile$','interpreter','latex','FontSize',20);
% xlabel('$Time (s)$','interpreter','latex','FontSize',20);
% ylabel('$q \: (rad)$','interpreter','latex','FontSize',20);

% figure 
% plot(t,xtraj_data(4,:),'o-.',t,xtraj_data(5,:),'o-',t,xtraj_data(6,:),'o-','linewidth',2)
% legend('Passive Joint 1','Active Joint 2','Active Joint 3')
% title('Generalized Velocity Profile');
% %title('$Joint\:Effort\: Profile$','interpreter','latex','FontSize',20);
% xlabel('$Time (s)$','interpreter','latex','FontSize',20);
% ylabel('$\dot{q} \: (rad/s)$','interpreter','latex','FontSize',20);

%  figure 
%  hold on
% F=fnplt(xtraj(1,:)); 
%               F.Color= [0 0.4470 0.7410];
%           F.LineStyle= '-.';
%           F.LineWidth= 2;
%             % F.Marker= 'o';
%          %F.MarkerSize= 6;
%          
%  F=fnplt(xtraj(2,:));
%           F.Color= [0.8500 0.3250 0.0980];
%           F.LineStyle= '-';
%           F.LineWidth= 2;
%             % F.Marker= 'o';
%          %F.MarkerSize= 6;
%          
%  F=fnplt(xtraj(3,:));
%  
%            F.Color= [0.9290 0.6940 0.1250];
%           F.LineStyle= '-';
%           F.LineWidth= 2;
%             % F.Marker= 'o';
%          %F.MarkerSize= 6;
% 
%          
% 
%  legend('Passive Joint 1','Active Joint 2','Active Joint 3')
%  xlabel('$Time (s)$','interpreter','latex','FontSize',20);
%  ylabel('$q \: (rad)$','interpreter','latex','FontSize',20);
%  title('Generalized Configuration Profile');
 
 
%%%%%%%%%%%%%%%%%%%VELOCITY%%%%%%%
figure 
 hold on
F=fnplt(xtraj(3,:)); 
              F.Color= [0 0.4470 0.7410];
          F.LineStyle= '-.';
          F.LineWidth= 2;
            % F.Marker= 'o';
         %F.MarkerSize= 6;
         
 F=fnplt(xtraj(4,:));
          F.Color= [0.8500 0.3250 0.0980];
          F.LineStyle= '-';
          F.LineWidth= 2;
            % F.Marker= 'o';
         %F.MarkerSize= 6;
         
 F=fnplt(xtraj(5,:));
 
           F.Color= [0.9290 0.6940 0.1250];
          F.LineStyle= '-';
          F.LineWidth= 2;
            % F.Marker= 'o';
         %F.MarkerSize= 6;
         
          legend('Passive Joint 1','Active Joint 2','Active Joint 3')
 xlabel('$Time (s)$','interpreter','latex','FontSize',20);
 ylabel('$\dot{q} \: (rad/s)$','interpreter','latex','FontSize',20);
 title('Generalized Velocity Profile');

 


