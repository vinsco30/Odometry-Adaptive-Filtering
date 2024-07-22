#include "../include/AKF_pkg/AKF_ros.h"

AKF_ros::AKF_ros() {

    if( !_nh.getParam("num_states", _states) ) {
        _states = 21;
    }
    if( !_nh.getParam("num_inputs", _inputs) ) {
        _inputs = 3;
    }
    if( !_nh.getParam("tau", _tau) ) {
        _tau = 0.1;
    }
    if( !_nh.getParam("Delta_t", _Dt) ) {
        _Dt = 0.01;
    }   
    if( !_nh.getParam("lambda_x", _lambda_x) ) {
        _lambda_x = 10.0;
    }  
    if( !_nh.getParam("lambda_y", _lambda_y) ) {
        _lambda_y = 10.0;
    }  
    if( !_nh.getParam("lambda_z", _lambda_z) ) {
        _lambda_z = 160.0;
    }  

    _nh.getParam( "kx", _kx );
    _nh.getParam( "ky", _ky );
    _nh.getParam( "kz", _kz );
    _nh.getParam( "epsilon", _epsilon );
    _nh.getParam( "q_v_meas_l_ok", _q_v_meas_l_ok );
    _nh.getParam( "q_l_meas_bad", _q_l_meas_bad );
    _nh.getParam( "r_l_bad", _r_l_bad );


    //---Inputs
    _lio_odom_sub = _nh.subscribe<nav_msgs::Odometry>( "/aft_mapped_to_init", 1, &AKF_ros::LIO_cb, this );
    _acc_ctrl_sub = _nh.subscribe<mrs_msgs::EstimatorInput>( "/uav1/control_manager/estimator_input", 1, &AKF_ros::ctrl_acc_cb, this );
    // _ref_ctrl_sub = _nh.subscribe<nav_msgs::Odometry>( "/uav1/control_manager/control_reference", 1, &AKF_ros::ctrl_ref_cb, this );
    _eig_sub = _nh.subscribe<std_msgs::Float32MultiArray>( "/point_lio/eig", 1, &AKF_ros::eig_cb, this );
    _second_source_sub = _nh.subscribe<nav_msgs::Odometry>( "/uav1/hw_api/odometry", 1, &AKF_ros::second_odom_cb, this );
    _points_sub = _nh.subscribe<std_msgs::UInt16>( "/point_lio/n_points", 1, &AKF_ros::points_cb, this );

    //--Output
    _robot_est = _nh.advertise<nav_msgs::Odometry>( "/AKF/odom", 1000 );

    //---Initialization
    _lio_odom_msg_received = false; 
    _ii_odom_msg_received = false;
    _new_ctrl_acc = false;
    _points_received = false;
    _first_meas = true;
    _init_kf = false;
    _meas_l_ok << true, true, true;

}

void AKF_ros::LIO_cb( const nav_msgs::Odometry lio_msg ) {

    _uav_pos << lio_msg.pose.pose.position.x, lio_msg.pose.pose.position.y, lio_msg.pose.pose.position.z;
    _uav_quat << lio_msg.pose.pose.orientation.w, lio_msg.pose.pose.orientation.x, lio_msg.pose.pose.orientation.y, lio_msg.pose.pose.orientation.z;
    _uav_vel << lio_msg.twist.twist.linear.x, lio_msg.twist.twist.linear.y, lio_msg.twist.twist.linear.z;
    _uav_ang_vel << lio_msg.twist.twist.angular.x, lio_msg.twist.twist.angular.y, lio_msg.twist.twist.angular.z;

    _z_l.block<3,1>(0,0) = _uav_pos;
    _z_l.block<3,1>(3,0) = _uav_vel;
    if( _first_meas && _uav_pos[2] >= 1.0 ) {

        ROS_INFO("Take-Off completed. Kalman filter init.");
        _init_kf = true;
        _first_meas = false;

    }
    /*Hysteresis*/
    //x
    std::cout<<_meas_l_ok[0]<<"\n";
    if( !_meas_l_ok[0] && _eig_xyz[0] > _lambda_x+_epsilon[0] ) {
        ROS_WARN("Meas x OK after");
        _meas_l_ok[0] = true;
    }
    else if( _meas_l_ok[0] && _eig_xyz[0] < _lambda_x ) {
        _meas_l_ok[0] = false;
        ROS_ERROR("Meas x BAD");
    }
    //y
    if( !_meas_l_ok[1] && _eig_xyz[1] > _lambda_y+_epsilon[1] ) {
        _meas_l_ok[1] = true;
    }
    else if( _meas_l_ok[1] && _eig_xyz[1] < _lambda_y ) {
        _meas_l_ok[1] = false;
    }
    //z
    if( !_meas_l_ok[2] && _eig_xyz[2] > _lambda_z+_epsilon[2] ) {
        _meas_l_ok[2] = true;
    }
    else if( _meas_l_ok[2] && _eig_xyz[2] < _lambda_z ) {
        _meas_l_ok[2] = false;
    }

    _lio_odom_msg_received = true;
}   

void AKF_ros::second_odom_cb( const nav_msgs::Odometry so_msg ) {

    _pose_gt << so_msg.pose.pose.position.x, so_msg.pose.pose.position.y, so_msg.pose.pose.position.z;
    
    _vel_gt << so_msg.twist.twist.linear.x, so_msg.twist.twist.linear.y, so_msg.twist.twist.linear.z;
    
    _z_v.block<3,1>(0,0) = _pose_gt;
    _z_v.block<3,1>(3,0) = _vel_gt;
    _ii_odom_msg_received = true;
}   

void AKF_ros::ctrl_acc_cb( const mrs_msgs::EstimatorInput ref_acc ) {

    _cmd_acc << ref_acc.control_acceleration.x, ref_acc.control_acceleration.y, ref_acc.control_acceleration.z;
    _new_ctrl_acc = true;
}

void AKF_ros::eig_cb( const std_msgs::Float32MultiArray eig_msg ) {

    // if( _points_received ) {
    //     _eig_xyz << eig_msg.data[6]/_points,  eig_msg.data[7]/_points, eig_msg.data[8]/_points;
    //     _points_received = false;
    // }
    // else {
        _eig_xyz << eig_msg.data[6],  eig_msg.data[7], eig_msg.data[8];
    // }
    
    _eig_received = true;
}

void AKF_ros::points_cb( const std_msgs::UInt16 points_msg ) {

    _points << points_msg.data;

    _points_received = true;
}

void AKF_ros::AKF_creation( Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& H_l, Eigen::MatrixXd& H_v,
        Eigen::MatrixXd& Q, Eigen::MatrixXd& R_l, Eigen::MatrixXd& R_v ) {

            A = Matrix<double, 21, 21>::Zero();
            B = Matrix<double, 21, 3>::Zero();
            H_l = Matrix<double, 6, 21>::Zero();
            H_v = Matrix<double, 6, 21>::Zero();
            Q = Matrix<double, 21, 21>::Zero();
            R_l = Matrix<double, 6,6>::Zero();
            R_v = Matrix<double, 6,6>::Zero();

            double a = exp(-_Dt/_tau);
            A = sys_model::build_A( a, _Dt );
            B = sys_model::build_B( a );
            H_l = sys_model::build_Hl();
            H_v = sys_model::build_Hv();
            Q = sys_model::build_Q();
            R_l = sys_model::build_R();
            R_v = sys_model::build_R();

            // for(int i=0; i<6; i++) {
            //     for(int j=0; j<6; j++) {
            //         std::cout<<R_l(i,j)<<" ";
            //     }
            //     std::cout<<"\n";
            // }

        }

void AKF_ros::fusion_loop() {

    ros::Rate r( 80 );
    Eigen::Matrix<double,21, 21> A;
    Eigen::Matrix<double,21,3> B;
    Eigen::Matrix<double,6, 21> H_L;
    Eigen::Matrix<double,6,21> H_V;
    Eigen::Matrix<double,21,21> Q;
    Eigen::Matrix<double,6,6> R_L;
    Eigen::Matrix<double,6,6> R_V;

    // AKF_creation(A, B, H_L, H_V, Q, R_L, R_V);
    double a = exp(-_Dt/_tau);
    A = sys_model::build_A( a, _Dt );
    B = sys_model::build_B( a );
    H_L = sys_model::build_Hl();
    H_V = sys_model::build_Hv();
    Q = sys_model::build_Q();
    R_L = sys_model::build_R();
    R_V = sys_model::build_R();

    /*KF initialization*/
    Eigen::Matrix<double, 21, 21> P_p = Matrix<double, 21, 21>::Identity();
    Eigen::Matrix<double, 21, 21> P_u = Matrix<double, 21, 21>::Identity();
    Eigen::Matrix<double, 21, 1> x_p = Matrix<double, 21, 1>::Zero();
    Eigen::Matrix<double, 21, 1> x_u = Matrix<double, 21, 1>::Zero();
    Eigen::Matrix<double, 6, 1> y = Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 21, 6> K = Matrix<double, 21, 6>::Zero();

    Eigen::Vector3d u;

    nav_msgs::Odometry odom_out_msg;

    while( ros::ok() ) {
        if( _init_kf ) {
            _init_kf = false;
            x_p << _uav_pos[0], _uav_pos[1], _uav_pos[2], _uav_vel[0], _uav_vel[1], _uav_vel[2];
            x_u = x_p;
            ROS_INFO("Kalman filter's states initialized!");

        }

        u = _cmd_acc;
        //--Predict
        x_p = A*x_u + B*u;
        P_p = A*P_u*A.transpose() + _Dt*Q;

        //--Correct
        y = _z_l - H_L*x_p;
        K = P_p * H_L.transpose() * (H_L * P_p * H_L.transpose() + R_L).inverse();

        x_u = x_p + K*y;
        P_u = (Eigen::Matrix<double,21,21>::Identity() - K * H_L) * P_p;

        odom_out_msg.pose.pose.position.x = x_u[0];
        odom_out_msg.pose.pose.position.y = x_u[1];
        odom_out_msg.pose.pose.position.z = x_u[2];
        odom_out_msg.twist.twist.linear.x = x_u[3];
        odom_out_msg.twist.twist.linear.y = x_u[4];
        odom_out_msg.twist.twist.linear.z = x_u[5];

        _robot_est.publish(odom_out_msg);

        

        r.sleep();
    }
}

void AKF_ros::run() {
    boost::thread check_drift_t( &AKF_ros::fusion_loop, this );
    ros::spin();
}

int main( int argc, char** argv ) {

    ros::init(argc, argv, "akf_ros");
    AKF_ros akf;
    akf.run();
    return 0;
}