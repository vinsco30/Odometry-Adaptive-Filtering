# Odometry Adaptive Fusion


## Parameters


## Inputs 
In the ```AKF.yaml``` file it's possible to set the inputs of the AKF filter:

1. ```first_odom_topic_name``` is the ekf based odometry (e.g. Point-LIO)

2. ```second_odom_topic_name``` the auxiliary odometry message (e.g. Open-Vins)

3. ```first_ekf_eig_topic_name``` Float32MultiArray message publishing the eigenvalues of the ekf odometry's information matrix (in the Point-LIO degradation branch is /point_lio/eig)

4. ```cmd_acc_topic_name``` mrs_msgs::EstimatorInput message which includes the commanded acceleration to the FCU.

## Outputs

1. ```/AKF/odom``` is the fused odometry message

2. ```/AKF/state_x``` and ```/AKF/state_y``` are boolean messages which returns the status of the hysteresis scheme