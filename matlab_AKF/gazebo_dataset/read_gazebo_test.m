clear 
close all
clc

bag = rosbag("test_AKF_2d_tuning_xy9.bag");
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
state_x_def = state_x_def*25;
state_y_def = state_y_def*25;

rapp_m = minL/length(eig_x);
fraction_m = sym(rapp_m);
[num_m, den_m] = numden(fraction_m);
l_m = double(num_m);
f_m = double(den_m);
eig_x_def = resample(eig_x,l_m,f_m);
eig_y_def = resample(eig_y,l_m,f_m);
n_points_def = resample(n_points,l_m,f_m);

% Plotting
figure
subplot(2,2,[1 2])
plot(x_lio_def,'r',LineWidth=3);
hold on
plot(x_gt_def,'g',LineWidth=3);
plot(x_akf_def,'--b',LineWidth=3);
legend('x_{LIO}', 'x_{gt}', 'x_{AKF}')
title('Odometry output x-axis')
subplot(2,2,[3 4])
plot(eig_x_def,'b', LineWidth=2);
hold on
plot(state_x_def, 'r', LineWidth=2);
title('Eigenvalue x-axis')
legend('eig_x', 'degraded_x')
% subplot(2,2,4)
% plot(n_points_def,'b',LineWidth=2);
% title('Number of points LIO')

figure
subplot(2,2,[1 2])
plot(y_lio_def,'r',LineWidth=3);
hold on
plot(y_gt_def,'g',LineWidth=3);
plot(y_akf_def,'--b',LineWidth=3);
legend('y_{LIO}', 'y_{gt}', 'y_{AKF}')
title('Odometry output y-axis')
subplot(2,2,[3 4])
plot(eig_y_def,'b', LineWidth=2);
hold on
plot(state_y_def, 'r', LineWidth=2);
legend('eig_y', 'degraded_y')
title('Eigenvalue y-axis')
% subplot(2,2,4)
% plot(n_points_def,'b',LineWidth=2);
% title('Number of points LIO')

figure
plot(x_lio_def,y_lio_def, 'r',LineWidth=2);
hold on
plot(x_gt_def,y_gt_def, 'g', LineWidth=2);
plot(x_akf_def, y_akf_def, 'b', LineWidth=2);
legend('p_{LIO}', 'p_{gt}', 'p_{AKF}')
title('x-y plane odometry estimation')



