close all

%% generate/obtain data to plot
u_hist = csvread('data/control_hist2.csv');
time = u_hist(:,1);
u_approx = u_hist(:,2); 

dt = time(2)-time(1);

state = zeros(4,length(time));
state(:,1) = [-10000;5000;150;-20];

for i = 2:length(time)
   t = time(i-1);
   x = state(:,i-1);
   u = u_approx(i-1);
   state(:,i) = x + dt*dynamics_ex2(t,x,u);
end


% plot results
figure(1)
plot(state(1,:),state(2,:),'-','Color',[0.5,0,1.0],'LineWidth',2)
grid on
xlabel('X','FontSize',16)
ylabel('Y','FontSize',16)

figure(2)
plot(time, state(3,:),'-','Color',[0.5,0,1.0],'LineWidth',2)
hold on
plot(time, state(4,:),'-','Color',[0.1,0.7,1.0],'LineWidth',2)
grid on
xlabel('Time','FontSize',16)
ylabel('Velocity','FontSize',16)
legend({'$v_{x}$','$v_{y}$'},'interpreter','latex','Location','Best')

figure(3)
plot(time,u_approx(:,1),'-','Color',[0.1,0.7,0.3],'LineWidth',2)
grid on

title('Control Plot','FontSize',16)
xlabel('Time (s)','FontSize',16)
ylabel('Control Value','FontSize',16)
legend({'$u_1$'},'interpreter','latex','Location','Best')
