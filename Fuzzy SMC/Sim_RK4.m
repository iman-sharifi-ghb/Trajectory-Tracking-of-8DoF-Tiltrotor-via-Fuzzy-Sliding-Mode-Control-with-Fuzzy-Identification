clc;clear;close all;

% X=x(1);X-dot=x(2);Y=x(3);Y-dot=x(4);Z=x(5);Z-dot=x(6);
% Phi=x(7);Phi-dot=x(8);Teta=x(9);Teta-dot=x(10);Psi=x(11);Psi-dot=x(12);

global m g l Ix Iy Iz 
global M1p M2p M3p M4p
global C1 C2 C3 C1p C2p C3p
global Kf Km 
global y u1s u2s u3s u4s
global ws Tilt1 Tilt2
global desiredStates
global slideSurfs
global Type
global fis1 fis2 fis3 fis4
global u_prev

u_prev = [10 10;0 0;0 0;0 0];

% fis1 = readfis('Ctrl1\u1.fis');
% fis2 = readfis('control\u2.fis');
fis3 = readfis('control\u3.fis');
% fis4 = readfis('control\u4.fis');

fis1 = readfis('Ctrl2\u1.fis');
fis2 = readfis('Ctrl2\u2.fis');
% fis3 = readfis('Ctrl2\u3.fis');
fis4 = readfis('Ctrl3\u4.fis');

Type = 3;
desiredStates = [];
slideSurfs = [];Xdd = [];
ws=[]; Tilt1=[]; Tilt2=[];
y = zeros(12,1);
u1s=[]; u2s=[]; u3s=[]; u4s=[];

% Evaluate Parameters
m = 0.468;g = 9.81;l = 0.225;Ix = 4.856e-3;Iy = 4.856e-3;Iz = 8.801e-3;
M1p = 0;M2p = 0;M3p = 0;M4p = 0;
C1 = 0.25;C2 = 0.25;C3 = 0.25;C1p = 0.01;C2p = 0.01;C3p = 1e-3;
Kf = 2.98e-6;Km = 1.14e-7; 

% Time
dt = 0.01;
Ti = 0; Tf = 20;
t = Ti:dt:Tf;

% Initial Conditions
x0 = [0.5;0;0.5;0;0.5;0
      0;0;0;0;0;0];
n = length(x0);
N = (Tf - Ti)/dt;

% Initialize
x = zeros(N,n);
k = zeros(4,n);
x(1,:) = x0.';
y_previous = zeros(12,1);
w = 621*ones(4,1);

%% Solve Equations
for j=1:N

    % RK4 Method------------------------------------------
    % RK4 Coeficients
    k(1,:) = dt*NonLinSystem(t(j), x(j,:), w, y_previous).' ;
    k(2,:) = dt*NonLinSystem(t(j)+0.5*dt, x(j,:)+0.5*k(1,:), w, y_previous).';
    k(3,:) = dt*NonLinSystem(t(j)+0.5*dt, x(j,:)+0.5*k(2,:), w, y_previous).';
    k(4,:) = dt*NonLinSystem(t(j)+dt, x(j,:)+k(3,:), w, y_previous).';
    
    % Update vector X 
    x(j+1,:) = x(j,:) + (k(1,:)+2*k(2,:)+2*k(3,:)+k(4,:))/6;
    y_previous = NonLinSystem(t(j), x(j,:), w, y_previous);
    Xdd = [Xdd; y_previous];
    
%     if isnan(x(j,5))
%         break
%     end
    
end
Xdd = Xdd(:,2:2:end);  
% options= odeset('Reltol',0.001,'Stats','on');
% [t,x] = ode45(@(t,x) TiltrotorODE(t,x), tspan, x0, options);

% Refine Control Inputs, Omega and Tilts
a = round(length(u1s)/length(x));
i=1;
ws_new=[];
u1s_new=[];u2s_new=[];u3s_new=[];u4s_new=[];
Tilt1_new=[];Tilt2_new=[];
slideSurfs_new = [];
desiredStates_new = [];
for k=1:length(u1s)
    if mod(k,a)==1
        slideSurfs_new(i,:) = slideSurfs(k,:);
        desiredStates_new(i,:) = desiredStates(k,:);
        u1s_new(i)= u1s(k); u2s_new(i)= u2s(k);
        u3s_new(i)= u3s(k); u4s_new(i)= u4s(k);
        Tilt1_new(i)= Tilt1(k); Tilt2_new(i)= Tilt2(k);
        i = i+1;
    end
end

%% Plot Results
figure;
title('Slide Surfaces')
xit = [1:length(slideSurfs_new)]*dt;
subplot(221);imanPlot(xit, slideSurfs_new(:,1),'S_1');
subplot(222);imanPlot(xit, slideSurfs_new(:,2),'S_2');
subplot(223);imanPlot(xit, slideSurfs_new(:,3),'S_3','Time');
subplot(224);imanPlot(xit, slideSurfs_new(:,4),'S_4','Time');
% legend('S_1','S_2','S_3','S_4')

figure;
title('Control Inputs')
subplot(221);imanPlot(xit, u1s_new,'u_1');
subplot(222);imanPlot(xit, u2s_new,'u_2');
subplot(223);imanPlot(xit, u3s_new,'u_3','Time');
subplot(224);imanPlot(xit, u4s_new,'u_4','Time');

%%
figure;
t1 = Tilt1_new(1:end);
t2 = Tilt2_new(1:end);
title('Tilt Inputs')
subplot(211);imanPlot(xit, rad2deg(t1),'\theta_1');
subplot(212);imanPlot(xit, rad2deg(t2),'\theta_2','Time');

%% 3D Position
figure
imanPlot(t,x(:,1),'-b');hold on
imanPlot(t,x(:,3),'-r');
imanPlot(t,x(:,5),'-g')
grid on;title('Position Response');legend('X','Y','Z')

%% Position Tracking
figure;grid on
plot3(x(:,1),x(:,3),x(:,5),'linewidth',2);hold on
plot3(desiredStates_new(:,1),desiredStates_new(:,4),desiredStates_new(:,7),'linewidth',2)
xlabel('X');ylabel('Y');zlabel('Z');grid on;title('3D Position Tracking')
legend('desired trajectory','Output Trajectory')
%% Attitude Tracking
figure;hold on
subplot(311);imanPlot(t,rad2deg(x(:,7)));hold on
plot(t(1:end-1),rad2deg(desiredStates_new(:,10)),'linewidth',2)
title('Attitude Tracking');ylabel('\phi')
legend('\phi','\phi_d_e_s')
subplot(312);imanPlot(t,rad2deg(x(:,9)));hold on
plot(t(1:end-1),rad2deg(desiredStates_new(:,13)),'linewidth',2)
ylabel('\theta');legend('\theta','\theta_d_e_s')
subplot(313);imanPlot(t,rad2deg(x(:,11)));hold on
plot(t(1:end-1),rad2deg(desiredStates_new(:,16)),'linewidth',2)
xlabel('Time');ylabel('\psi');legend('\psi','\psi_d_e_s')

%% Video Animation
% close all
% tic;
% figure;
% x_des = desiredStates_new(:,1);
% y_des = desiredStates_new(:,4);
% z_des = desiredStates_new(:,7);
% Quad_Animation(t,x(:,1),x(:,3),x(:,5),x(:,7),x(:,9),x(:,11),x_des,y_des,z_des)
% toc

