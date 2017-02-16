% Author: C. Howard
% plot results of adjoint solution for the following example problem 1:
%
% 

%% cleanup anything that might exist
clear all
close all

%% generate/obtain data to plot
u_hist = csvread('data/control_hist2.csv');
time = u_hist(:,1);
u_approx = u_hist(:,2); 

%% plot the results
figure(1)
plot(time,u_approx(:,1),'-','Color',[0.1,0.7,0.3],'LineWidth',2)
%hold on
%plot(time,u_approx(:,2),'-.','Color',[0.5,0,1.0],'LineWidth',2)
grid on

title('Control Plot','FontSize',14)
xlabel('Time (s)','FontSize',14)
ylabel('Control Value','FontSize',14)
% legend({'$u_1$','$u_2$'},'interpreter','latex','Location','Best')
legend({'$u_1$'},'interpreter','latex','Location','Best')