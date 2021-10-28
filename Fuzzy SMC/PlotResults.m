clc;clear;%close all

SMC  = load('SMC1.mat');
FSMC = load('FSMC1.mat');

t = SMC.t;


%% Plot Results
% figure;
% subplot(211);plot(u1s_new(1:end),'linewidth',2);grid on
% legend('u_1');title('Control Inputs');
% subplot(212);plot(u2s_new(1:end),'-r','linewidth',2);hold on
% plot(u3s_new(1:end),'-g','linewidth',2);hold on
% plot(u4s_new(1:end),'-y','linewidth',2)
% xlabel('Iteration');legend('u_2','u_3','u_4')
% grid on

%%
% figure
% t1 = Tilt1_new(1:end);
% t2 = Tilt2_new(1:end);
% % subplot(211);imanPlot([1:length(ws_new)],ws_new,'Omega')subplot(212);
% imanPlot([1:length(t2)],rad2deg(t1))
% hold on;imanPlot([1:length(t2)],rad2deg(t2),'Tilt Inputs')
% legend('\theta_1','\theta_2')
% xlabel('Iteration')

%%
% figure;
% imanPlot(t,x(:,1),'-b');hold on
% imanPlot(t,x(:,3),'-r');
% imanPlot(t,x(:,5),'-g')
% grid on;title('Position Response');legend('X','Y','Z')

%%
figure;
subplot(311);plot(t,SMC.x(:,1),'b-','linewidth',3);hold on
plot(t,FSMC.x(:,1),'k-.','linewidth',3);hold on
plot(t(1:end-1), SMC.desiredStates_new(:,1),'r--','LineWidth',3)
title('Position Response'),ylabel('X(m)')
grid on
legend('SMC','FSMC','X_d_e_s')

subplot(312);plot(t,SMC.x(:,3),'b-','linewidth',3);hold on
plot(t,FSMC.x(:,3),'k-.','linewidth',3);hold on
plot(t(1:end-1), SMC.desiredStates_new(:,4),'r--','LineWidth',3)
grid on
ylabel('Y(m)'),legend('SMC','FSMC','Y_d_e_s')

subplot(313);plot(t,SMC.x(:,5),'b-','linewidth',3);hold on
plot(t,FSMC.x(:,5),'k-.','linewidth',3);hold on
plot(t(1:end-1), SMC.desiredStates_new(:,7),'r--','LineWidth',3)
xlabel('Time'),ylabel('Z(m)')
grid on
legend('SMC','FSMC','Z_d_e_s')

%% Attitude Tracking
figure;hold on
subplot(311);plot(t,rad2deg(SMC.x(:,7)),'b-','linewidth',3);hold on
plot(t,rad2deg(FSMC.x(:,7)),'k-.','linewidth',3);hold on
plot(t(1:end-1),rad2deg(SMC.desiredStates_new(:,10)),'r--','linewidth',3)
title('Attitude Tracking');ylabel('\phi(deg)')
grid on
legend('SMC','FSMC','\phi_d_e_s')

subplot(312);plot(t,rad2deg(SMC.x(:,9)),'b-','linewidth',3);hold on
plot(t,rad2deg(FSMC.x(:,9)),'k-.','linewidth',3);hold on
plot(t(1:end-1),rad2deg(SMC.desiredStates_new(:,13)),'r--','linewidth',3)
grid on
ylabel('\theta(deg)');legend('SMC','FSMC','\theta_d_e_s')

subplot(313);plot(t,rad2deg(SMC.x(:,11)),'b-','linewidth',3);hold on
plot(t,rad2deg(FSMC.x(:,11))+0.004,'k-.','linewidth',3);hold on
plot(t(1:end-1),rad2deg(SMC.desiredStates_new(:,16)),'r--','linewidth',3)
grid on
xlabel('Time');ylabel('\psi(deg)');legend('SMC','FSMC','\psi_d_e_s')


%% Position Tracking
figure;grid on
plot3(SMC.x(:,1),SMC.x(:,3),SMC.x(:,5),'b-','linewidth',3);hold on
plot3(FSMC.x(:,1),FSMC.x(:,3),FSMC.x(:,5),'k-.','linewidth',3);hold on 
plot3(SMC.desiredStates_new(:,1),SMC.desiredStates_new(:,4),SMC.desiredStates_new(:,7),'r--','linewidth',3)
xlabel('X');ylabel('Y');zlabel('Z');grid on;title('3D Position Tracking')
legend('SMC Trajectory','FSMC trajectory','desired trajectory')


