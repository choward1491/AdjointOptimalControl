% Author: C. Howard
% plot results of adjoint solution for the following example problem 1:
%
% dynamics:
%   xdot = f(t,x,u) = -x + u
% integral cost:
%   L(t,x,u) = 0.5*(x^2 + u^2)
% final cost:
%   Psi(x(tf)) = 0
% 

%% cleanup anything that might exist
clear all
close all

%% generate/obtain data to plot
u_hist = csvread('data/control_hist.csv');
time = u_hist(:,1);
u_approx = u_hist(:,2);
u_exact = sinh(sqrt(2)*(time-1))./(sqrt(2)*cosh(sqrt(2)) + sinh(sqrt(2))); 

%% plot the results
figure(1)
plot(time,u_exact,'-','Color',[0.1,0.7,0.3],'LineWidth',2)
hold on
plot(time,u_approx,'-.','Color',[0.5,0,1.0],'LineWidth',2)
grid on

title('Control Comparison','FontSize',14)
xlabel('Time (s)','FontSize',14)
ylabel('Control Value','FontSize',14)
legend({'$u$','$\hat{u}$'},'interpreter','latex','Location','Best')