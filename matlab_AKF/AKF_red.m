clear
close all

%% KF creation for one dimension
Dt = 1e-1;
tau1 = 1e-1;

a = exp(-Dt/tau1);

A = [1 Dt Dt^2/2; 0 1 Dt; 0 0 a];

B = [0; 0; 1-a];

% H_L = [1 0 0 -1 0 0; 0 1 0 0 -1 0]; %problema nel modello di misura
H_L = [1 0 0; 0 1 0];
H_V = [1 0 0 0 -1 0];

qr = [1, 1, 1]*0.1;
qd = [1, 1, 1]*0.1;
% Q = zeros(21,21,size(ax_ref_def,1));
Q = zeros(3,3);
Q(:,:) = [qr.*eye(3)];

r_l = [0.1, 0.01]*1e-1;

R_L = r_l.*eye(2);
% R_V = r_v.*eye(2);

%Load plot_odom_ros.m 
plot_odom_ros
%Compute statistics
test_analyzer

u = ax_ref_def';
z_l = [X', vx'];
% z_l = X';
z_v = x_gt_def';
t = size(u,2);
x_p = zeros(3,t+1);
x_p(1,1) = x_gt_def(1,1)
x_s = zeros(3,t+1);
x_s(:,1) = x_p(:,1)
P_p = zeros(3,3,t+1);
P_p(:,:,1) = 0.01*eye(3)
P_s = zeros(3,3,t+1);
P_s(:,:,1) = 0.01*eye(3)
y = zeros(2,t+1);
y(:,1) = z_l(:,1) - H_L*x_p(:,1)
K = zeros(3,2,t+1);
K(:,:,1) = P_p(:,:,1)*H_L'*inv(H_L*P_p(:,:,1)*H_L' + R_L)

% eig_x = eig_x_def';

err = zeros(1,t);

for i=2:size(u,2)

    x_p(:,i) = A*x_s(:,i-1) + B*u(:,i-1)
    P_p(:,:,i) = A*P_s(:,:,i-1)*A' + Dt*Q(:,:)


    y(:,i) = z_l(:,i) - H_L*x_p(:,i)
    K(:,:,i) = P_p(:,:,i)*H_L'*inv(H_L*P_p(:,:,i)*H_L' + R_L)
    K(2,2,i);
    % end
    % K(:,:,i) = P_p(:,:,i)*H'*inv(H*P_p(:,:,i)*H' + R_L);
    x_s(:,i) = x_p(:,i) + K(:,:,i)*y(:,i)
    P_s(:,:,i) = (eye(3)-K(:,:,i)*H_L)*P_p(:,:,i)
    err(1,i) = abs(x_s(1,i)-x_gt_def(i));

end

figure('Renderer', 'painters', 'Position', [10 10 900 600])
subplot(3,1,1)
plot(t1,x_s(1,1:end-1),'Color','[0.07,0.62,1.00]',LineWidth=2')
hold on
plot(t1,X,'--','Color','[0.07,0.62,1.00]',LineWidth=2)
plot(t1,x_gt_def,':','Color','[0.07,0.62,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$x_{sys}$','interpreter','latex','Location','northeastoutside')
ylabel('${x}$ $[m]$','fontsize',18, 'interpreter','latex')
subplot(3,1,2)
plot(t1,x_s(2,1:end-1),'Color','[1.00,0.41,0.16]',LineWidth=2)
hold on
plot(t1,vx,'--','Color','[1.00,0.41,0.16]',LineWidth=2)
plot(t1,vx_gt_def,':','Color','[1.00,0.41,0.16]',LineWidth=2)
set(gca,'fontsize',16)
legend('$y_{sys}$','interpreter','latex','Location','northeastoutside')
ylabel('${y}$ $[m]$','fontsize',18, 'interpreter','latex')
subplot(3,1,3)
plot(t1,x_s(3,1:end-1),'Color','[1.00,0.00,1.00]',LineWidth=2)
hold on
plot(t1,ax_ref_def,':','Color','[1.00,0.00,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$z_{sys}$','interpreter','latex','Location','northeastoutside')
xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
ylabel('${z}$ $[m]$','fontsize',18, 'interpreter','latex')