clear 
close all
clc

bag = rosbag("thesis/forest_to_of5.bag");
bSel = select(bag,'Topic','/aft_mapped_to_init');
bSel1 = select(bag,'Topic','/uav1/hw_api/odometry');
bSel2 = select(bag,'Topic','/AKF/odom');
bSel3 = select(bag,'Topic','/AKF/state_x');
bSel4 = select(bag,'Topic','/AKF/state_y');

bMetrics = select(bag, 'Topic','/point_lio/eig');
bMetrics2 = select(bag, 'Topic','/point_lio/n_points');
bMetrics3 = select(bag, 'Topic','/point_lio/trace');

msgStructs = readMessages(bSel,'DataFormat','struct');
msgStructs1 = readMessages(bSel1,'DataFormat','struct');
msgStructs2 = readMessages(bSel2,'DataFormat','struct');
msgStructs3 = readMessages(bSel3,'DataFormat','struct');
msgStructs4 = readMessages(bSel4,'DataFormat','struct');

msgM = readMessages(bMetrics,'DataFormat','struct');
msgM2 = readMessages(bMetrics2,'DataFormat','struct');
msgM3 = readMessages(bMetrics3,'DataFormat','struct');

%Point LIO
x_lio = cellfun(@(m) double(m.Pose.Pose.Position.X), msgStructs);
y_lio = cellfun(@(m) double(m.Pose.Pose.Position.Y), msgStructs);
z_lio = cellfun(@(m) double(m.Pose.Pose.Position.Z), msgStructs);
vx_lio = cellfun(@(m) double(m.Twist.Twist.Linear.X), msgStructs);
vy_lio = cellfun(@(m) double(m.Twist.Twist.Linear.Y), msgStructs);
vz_lio = cellfun(@(m) double(m.Twist.Twist.Linear.Z), msgStructs);

% Second source
x_gt = cellfun(@(m) double(m.Pose.Pose.Position.X), msgStructs1);
y_gt = cellfun(@(m) double(m.Pose.Pose.Position.Y), msgStructs1);
z_gt = cellfun(@(m) double(m.Pose.Pose.Position.Z), msgStructs1);
vx_gt = cellfun(@(m) double(m.Twist.Twist.Linear.X), msgStructs1);
vy_gt = cellfun(@(m) double(m.Twist.Twist.Linear.Y), msgStructs1);
vz_gt = cellfun(@(m) double(m.Twist.Twist.Linear.Z), msgStructs1);

% AKF output
x_akf = cellfun(@(m) double(m.Pose.Pose.Position.X), msgStructs2);
y_akf = cellfun(@(m) double(m.Pose.Pose.Position.Y), msgStructs2);
vx_akf = cellfun(@(m) double(m.Twist.Twist.Linear.X), msgStructs2);
vy_akf = cellfun(@(m) double(m.Twist.Twist.Linear.Y), msgStructs2);
state_x = cellfun(@(m) double(m.Data), msgStructs3);
state_y = cellfun(@(m) double(m.Data), msgStructs4);


% Degradation metrics
eig_x = cellfun(@(m) double(m.Data(7,1)), msgM);
eig_y = cellfun(@(m) double(m.Data(8,1)), msgM);
eig_z = cellfun(@(m) double(m.Data(9,1)), msgM);
eig_velx = cellfun(@(m) double(m.Data(10,1)), msgM);
eig_vely = cellfun(@(m) double(m.Data(11,1)), msgM);
eig_velz = cellfun(@(m) double(m.Data(12,1)), msgM);
n_points = cellfun(@(m) double(m.Data), msgM2);
trace = cellfun(@(m) double(m.Data), msgM3);

% Downsampling
minL = [min([length(x_lio),length(x_akf), length(x_gt), length(eig_x)])];

rapp_lio = minL/length(x_lio);
fraction_lio = sym(rapp_lio);
[num, den] = numden(fraction_lio);
ll = double(num);
ff = double(den);
x_lio_def = resample(x_lio,ll,ff);
y_lio_def = resample(y_lio,ll,ff);

rapp_gt = minL/length(x_gt);
fraction_gt= sym(rapp_gt);
[num_gt, den_gt] = numden(fraction_gt);
l_gt = double(num_gt);
f_gt = double(den_gt);
x_gt_def = resample(x_gt,l_gt,f_gt);
y_gt_def = resample(y_gt,l_gt,f_gt);

rapp_akf = minL/length(x_akf);
fraction_akf = sym(rapp_akf);
[num_akf, den_akf] = numden(fraction_akf);
l_akf = double(num_akf);
f_akf = double(den_akf);
x_akf_def = resample(x_akf,l_akf,f_akf);
y_akf_def = resample(y_akf,l_akf,f_akf);
state_x_def = resample(state_x, l_akf, f_akf);
state_y_def = resample(state_y, l_akf, f_akf);
state_x_def = state_x_def*40;
state_y_def = state_y_def*40;

rapp_m = minL/length(eig_x);
fraction_m = sym(rapp_m);
[num_m, den_m] = numden(fraction_m);
l_m = double(num_m);
f_m = double(den_m);
eig_x_def = resample(eig_x,l_m,f_m);
eig_y_def = resample(eig_y,l_m,f_m);
th_x_sup = (10.6386+14)*ones(size(eig_x_def,1));
th_x_inf = (10.6386+6)*ones(size(eig_x_def,1));
th_y_sup = (11.4496+12)*ones(size(eig_y_def,1));
th_y_inf = (11.4496+0)*ones(size(eig_y_def,1));
% n_points_def = resample(n_points,l_m,f_m);


% eig_y_red = eig_y_def(400:800,1);

% 
% figure('Renderer', 'painters', 'Position', [10 10 900 600])
% % subplot(2,1,1)
% title('Eigenvalue y-axis', 'Interpreter', 'latex', 'FontSize', 20);
% plot(eig_y_red((1:end),1),'Color','[0.07,0.62,1.00]',LineWidth=2)
% hold on
% plot(th_y_sup_red((1:end),1),'--k',LineWidth=2)
% plot(th_y_inf_red((1:end),1),'--k',LineWidth=2)
% grid on
% % set(gca,'fontsize',16)
% % legend('$eig$','interpreter','latex','Location','northeastoutside')
% % % ylabel('${p_x}$ $[m]$','fontsize',18, 'interpreter','latex')
% % subplot(2,1,2)
% % plot(state_y_red((1:end),1),'Color','[1.00,0.41,0.16]',LineWidth=2)
% % set(gca,'fontsize',16)
% % legend('$y_status$','interpreter','latex','Location','northeastoutside')
% % % ylabel('${p_y}$ $[m]$','fontsize',18, 'interpreter','latex')
% xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
% grid on
% set(gca, 'TickLabelInterpreter', 'latex');

%Plot x-y
figure('Renderer', 'painters', 'Position', [10 10 900 600])

plot(x_lio_def,y_lio_def,'Color','[0.07,0.62,1.00]',LineWidth=2);
hold on

plot(x_akf_def,y_akf_def,'Color','[0.07,1.0,0.07]',LineWidth=2);
plot(x_gt_def,y_gt_def,'Color','[1.0,0.0,0.07]',LineWidth=2);
grid on
xlabel('$x$ $[m]$','fontsize',18,'interpreter','latex')
ylabel('$y$ $[m]$','fontsize',18, 'interpreter','latex')
xlim([-10 80]);
ylim([-10 60]);
title('x-y plane trajectory', 'Interpreter', 'latex', 'FontSize', 20);
set(gca, 'TickLabelInterpreter', 'latex');

% th_y_sup_red = th_y_sup(400:800,1);
% th_y_inf_red = th_y_inf(400:800,1);
% state_y_red = state_y_def(400:800,1)/25;
% 
% t1 = linspace(0,401,401);


% figure('Renderer', 'painters', 'Position', [10 10 900 600])
% title('Estimated trajectory', 'Interpreter', 'latex', 'FontSize', 20);
% subplot(4,2,[1 2])
% plot(x_lio_def,'Color','[0.07,0.62,1.00]',LineWidth=2);
% hold on
% plot(x_gt_def,'Color','[1.0,0.0,0.07]',LineWidth=2);
% plot(x_akf_def,'Color','[0.07,1.0,0.07]',LineWidth=2);
% xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
% ylabel('$x$ $[m]$','fontsize',18, 'interpreter','latex')
% ylim([-30 60]);
% grid on
% subplot(4,2,[5 6])
% plot(y_lio_def,'Color','[0.07,0.62,1.00]',LineWidth=2);
% hold on
% plot(y_gt_def,'Color','[1.0,0.0,0.07]',LineWidth=2);
% plot(y_akf_def,'Color','[0.07,1.0,0.07]',LineWidth=2);
% xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
% ylabel('$y$ $[m]$','fontsize',18, 'interpreter','latex')
% ylim([-30 40]);
% grid on
% subplot(4,2,[3 4])
% plot(eig_x_def,'Color','[0.07,0.62,1.00]',LineWidth=2);
% grid on
% hold on
% plot(th_x_sup,'--k',LineWidth=1)
% plot(th_x_inf,'--k',LineWidth=1)
% plot(state_x_def, 'r', LineWidth=1);
% subplot(4,2,[7 8])
% plot(eig_y_def,'Color','[0.07,0.62,1.00]',LineWidth=2);
% grid on
% hold on
% plot(th_y_sup,'--k',LineWidth=1)
% plot(th_y_inf,'--k',LineWidth=1)
% plot(state_y_def, 'r', LineWidth=1);

figure('Renderer', 'painters', 'Position', [10 10 900 600])
title('Estimated trajectory', 'Interpreter', 'latex', 'FontSize', 20);
subplot(2,2,1)
plot(x_lio_def,'Color','[0.07,0.62,1.00]',LineWidth=2);
hold on
plot(x_gt_def,'Color','[1.0,0.0,0.07]',LineWidth=2);
plot(x_akf_def,'Color','[0.07,1.0,0.07]',LineWidth=2);
% plot(y_gt_def,'Color','[1.0,0.0,0.07]',LineWidth=2);
% plot(y_akf_def,'Color','[0.07,1.0,0.07]',LineWidth=2);
% plot(y_lio_def,'Color','[0.07,0.62,1.00]',LineWidth=2);
% xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
ylabel('$x_p$ $[m]$','fontsize',18, 'interpreter','latex')
title('x trajectory', 'Interpreter', 'latex', 'FontSize', 20);
ylim([-30 60]);
grid on
set(gca, 'TickLabelInterpreter', 'latex');
subplot(2,2,3)
plot(eig_x_def,'Color','[0.07,0.62,1.00]',LineWidth=2);
hold on
grid on
plot(th_x_sup(1:end,1),LineWidth=1)
plot(th_x_inf(1:end,1),LineWidth=1)
plot(state_x_def, 'r', LineWidth=1);
title('x eigenvalue and state of the filter', 'Interpreter', 'latex', 'FontSize', 20);
xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
set(gca, 'TickLabelInterpreter', 'latex');
% plot(eig_y_def,'Color','[0.07,0.62,1.00]',LineWidth=2);
% plot(th_y_sup,'--k',LineWidth=1)
% plot(th_y_inf,'--k',LineWidth=1)

% figure('Renderer', 'painters', 'Position', [10 10 900 600])
% title('Estimated trajectory', 'Interpreter', 'latex', 'FontSize', 20);
subplot(2,2,2)
plot(y_lio_def,'Color','[0.07,0.62,1.00]',LineWidth=2);
hold on
plot(y_gt_def,'Color','[1.0,0.0,0.07]',LineWidth=2);
plot(y_akf_def,'Color','[0.07,1.0,0.07]',LineWidth=2);
% plot(y_gt_def,'Color','[1.0,0.0,0.07]',LineWidth=2);
% plot(y_akf_def,'Color','[0.07,1.0,0.07]',LineWidth=2);
% plot(y_lio_def,'Color','[0.07,0.62,1.00]',LineWidth=2);
% xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
ylabel('$y_p$ $[m]$','fontsize',18, 'interpreter','latex')
title('y trajectory', 'Interpreter', 'latex', 'FontSize', 20);
ylim([-30 60]);
grid on
set(gca, 'TickLabelInterpreter', 'latex');
subplot(2,2,4)
plot(eig_y_def,'Color','[0.07,0.62,1.00]',LineWidth=2);
hold on
grid on
plot(th_y_sup(1:end,1),LineWidth=1)
plot(th_y_inf(1:end,1),LineWidth=1)
plot(state_y_def, 'r', LineWidth=1);
title('y eigenvalue and state of the filter', 'Interpreter', 'latex', 'FontSize', 20);
xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
set(gca, 'TickLabelInterpreter', 'latex');


%Plot x and y separatelyy


% Get current axis ticksfigure('Renderer', 'painters', 'Position', [10 10 900 600])
% title('x-y plane trajectory', 'Interpreter', 'latex', 'FontSize', 20);
% plot(x_lio_def,y_lio_def,'Color','[0.07,0.62,1.00]',LineWidth=2);
% hold on
% plot(x_gt_def,y_gt_def,'Color','[1.0,0.0,0.07]',LineWidth=2);
% plot(x_akf_def,y_akf_def,'Color','[0.07,1.0,0.07]',LineWidth=2);
% grid on
% xlabel('$x$ $[m]$','fontsize',18,'interpreter','latex')
% ylabel('$y$ $[m]$','fontsize',18, 'interpreter','latex')
% set(gca, 'TickLabelInterpreter', 'latex');
% xticks = get(gca, 'XTick');
% yticks = get(gca, 'YTick');

% Create LaTeX-formatted tick labels (for example, using powers of 10)
% xtick_labels = arrayfun(@(x) sprintf('$%d$', x), xticks, 'UniformOutput', false);
% ytick_labels = arrayfun(@(y) sprintf('$%d$', y), yticks, 'UniformOutput', false);

% Set the custom tick labels
% set(gca, 'XTickLabel', xtick_labels);
% set(gca, 'YTickLabel', ytick_labels);
% 
% % Plotting
% figure
% subplot(2,2,[1 2])
% plot(x_lio_def,'r',LineWidth=3);
% hold on
% plot(x_gt_def,'g',LineWidth=3);
% plot(x_akf_def,'--b',LineWidth=3);
% legend('x_{LIO}', 'x_{gt}', 'x_{AKF}')
% title('Odometry output x-axis')
% subplot(2,2,[3 4])
% plot(eig_x_def,'b', LineWidth=2);
% hold on
% plot(state_x_def, 'r', LineWidth=2);
% plot(th_x_sup,'--k',LineWidth=2)
% plot(th_x_inf,'--k',LineWidth=2)
% title('Eigenvalue x-axis')
% legend('eig_x', 'degraded_x')
% % subplot(2,2,4)
% % plot(n_points_def,'b',LineWidth=2);
% % title('Number of points LIO')
% 
% figure
% subplot(2,2,[1 2])
% plot(y_lio_def,'r',LineWidth=3);
% hold on
% plot(y_gt_def,'g',LineWidth=3);
% plot(y_akf_def,'--b',LineWidth=3);
% legend('y_{LIO}', 'y_{gt}', 'y_{AKF}')
% title('Odometry output y-axis')
% subplot(2,2,[3 4])
% plot(eig_y_def,'b', LineWidth=2);
% hold on
% plot(state_y_def, 'r', LineWidth=2);
% plot(th_y_sup,'--k',LineWidth=2)
% plot(th_y_inf,'--k',LineWidth=2)
% legend('eig_y', 'degraded_y')
% title('Eigenvalue y-axis')
% % subplot(2,2,4)
% % plot(n_points_def,'b',LineWidth=2);
% % title('Number of points LIO')
% 
% figure
% plot(x_lio_def,y_lio_def, 'r',LineWidth=2);
% hold on
% plot(x_gt_def,y_gt_def, 'g', LineWidth=2);
% plot(x_akf_def, y_akf_def, 'b', LineWidth=2);
% legend('p_{LIO}', 'p_{gt}', 'p_{AKF}')
% title('x-y plane odometry estimation')



