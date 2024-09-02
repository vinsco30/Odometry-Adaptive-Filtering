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
    if( !_nh.getParam("debug", _debug) ) {
        _debug = false;
    } 
    if( !_nh.getParam("recovery", _do_reboot) ) {
        _do_reboot = false;
    } 
    if( !_nh.getParam("dist_threshold", _dist_th ) ) {
        _dist_th = 1.0;
    }
    if( !_nh.getParam("time_threshold", _time_th ) ) {
        _time_th = 5.0;
    }
    if( !_nh.getParam("node_name", _node_name) ) {
        _node_name = "/laserMapping";
    }

    _nh.getParam( "kx", _kx );
    _nh.getParam( "ky", _ky );
    _nh.getParam( "kz", _kz );
    _nh.getParam( "epsilon_bad", _epsilon_sup );
    _nh.getParam( "epsilon_ok", _epsilon_inf );
    _nh.getParam( "q_v_meas_l_ok", _q_v_meas_l_ok );
    _nh.getParam( "q_l_meas_bad", _q_l_meas_bad );
    _nh.getParam( "r_l_bad", _r_l_bad );

    if( !_nh.getParam( "first_odom_topic_name", _first_odom_topic_name ) ) {
        _first_odom_topic_name = "/aft_mapped_to_init";
    }
    if( !_nh.getParam( "second_odom_topic_name", _second_odom_topic_name ) ) {
        _second_odom_topic_name = "/uav1/hw_api/odometry";
    }
    if( !_nh.getParam( "first_ekf_eig_topic_name", _first_ekf_eig_topic_name ) ) {
        _first_ekf_eig_topic_name = "/point_lio/eig";
    }
    if( !_nh.getParam( "cmd_acc_topic_name", _cmd_acc_topic_name ) ) {
        _cmd_acc_topic_name = "/uav1/control_manager/estimator_input";
    }

    //---Inputs
    _lio_odom_sub = _nh.subscribe<nav_msgs::Odometry>( _first_odom_topic_name.c_str(), 1, &AKF_ros::LIO_cb, this );
    _acc_ctrl_sub = _nh.subscribe<mrs_msgs::EstimatorInput>( _cmd_acc_topic_name.c_str(), 1, &AKF_ros::ctrl_acc_cb, this );
    // _ref_ctrl_sub = _nh.subscribe<nav_msgs::Odometry>( "/uav1/control_manager/control_reference", 1, &AKF_ros::ctrl_ref_cb, this );
    _eig_sub = _nh.subscribe<std_msgs::Float32MultiArray>( _first_ekf_eig_topic_name.c_str(), 1, &AKF_ros::eig_cb, this );
    _second_source_sub = _nh.subscribe<nav_msgs::Odometry>( _second_odom_topic_name.c_str(), 1, &AKF_ros::second_odom_cb, this );
    _points_sub = _nh.subscribe<std_msgs::UInt16>( "/point_lio/n_points", 1, &AKF_ros::points_cb, this );
    _trace_sub = _nh.subscribe<std_msgs::Float32>( "/point_lio/trace", 1, &AKF_ros::trace_cb, this );

    //--Output
    _robot_est = _nh.advertise<nav_msgs::Odometry>( "/AKF/odom", 1000 );
    _filter_state_x = _nh.advertise<std_msgs::Bool>( "/AKF/state_x", 1000 );
    _filter_state_y = _nh.advertise<std_msgs::Bool>( "/AKF/state_y", 1000 );

    //---Initialization
    _lio_odom_msg_received = false; 
    _ii_odom_msg_received = false;
    _new_ctrl_acc = false;
    _points_received = false;
    _trace_received = false;
    _first_meas = true;
    _init_kf = false;
    _meas_l_ok << true, true, true;
    // _q_change_ok << false, false, false;
    _takeoff_done = false;
    _state_x = false;
    _state_y = false;
    _dist_max = false;
    _killed = false;

}

void AKF_ros::LIO_cb( const nav_msgs::Odometry lio_msg ) {

    _uav_pos << lio_msg.pose.pose.position.x, lio_msg.pose.pose.position.y, lio_msg.pose.pose.position.z;
    _uav_quat << lio_msg.pose.pose.orientation.w, lio_msg.pose.pose.orientation.x, lio_msg.pose.pose.orientation.y, lio_msg.pose.pose.orientation.z;
    _uav_vel << lio_msg.twist.twist.linear.x, lio_msg.twist.twist.linear.y, lio_msg.twist.twist.linear.z;
    _uav_ang_vel << lio_msg.twist.twist.angular.x, lio_msg.twist.twist.angular.y, lio_msg.twist.twist.angular.z;

    // std::cout<<_uav_pos[0]<<", "<<_uav_pos[1]<<", "<<_uav_pos[2]<<"\n";

    if( _first_meas && _uav_pos[2] >= 1.0 ) {

        ROS_INFO("Take-Off completed. Kalman filter init.");
        _init_kf = true;
        _first_meas = false;
        _takeoff_done=true;

    }
    /*Hysteresis*/
    if( _takeoff_done ) {
    //x
    // std::cout<<_meas_l_ok[0]<<"\n";
    if( !_meas_l_ok[0] && _eig_xyz[0] > _lambda_x+_epsilon_sup[0] ) {
        ROS_WARN("Meas x OK after");
        _meas_l_ok[0] = true;
        _q_change_ok[0] = false;
        _rq_change_bad[0] = false;
        _state_x=false;
    }
    else if( _meas_l_ok[0] && _eig_xyz[0] < _lambda_x+_epsilon_inf[0] ) {
        _meas_l_ok[0] = false;
        ROS_ERROR("Meas x BAD");
        _state_x = true;
    }
    //y
    if( !_meas_l_ok[1] && _eig_xyz[1] > _lambda_y+_epsilon_sup[1] ) {
        ROS_WARN("Meas y OK after");
        _meas_l_ok[1] = true;
        _q_change_ok[1] = false;
        _rq_change_bad[1] = false;
        _state_y = false;
    }
    else if( _meas_l_ok[1] && _eig_xyz[1] < _lambda_y+_epsilon_inf[1] ) {
        _meas_l_ok[1] = false;
         ROS_ERROR("Meas y BAD");
        _state_y = true;
    }
    //z
    if( !_meas_l_ok[2] && _eig_xyz[2] > _lambda_z+_epsilon_sup[2] ) {
        _meas_l_ok[2] = true;
        _q_change_ok[2] = false;
        _rq_change_bad[2] = false;
    }
    else if( _meas_l_ok[2] && _eig_xyz[2] < _lambda_z ) {
        _meas_l_ok[2] = false;
    }
    }

    // if( _killed ) {

    // }
    _z_l.block<3,1>(0,0) = _uav_pos;
    _z_l.block<3,1>(3,0) = _uav_vel;
    _lio_odom_msg_received = true;
}   

void AKF_ros::second_odom_cb( const nav_msgs::Odometry so_msg ) {

    _pose_gt << so_msg.pose.pose.position.x, so_msg.pose.pose.position.y, so_msg.pose.pose.position.z;
    
    _vel_gt << so_msg.twist.twist.linear.x, so_msg.twist.twist.linear.y, so_msg.twist.twist.linear.z;

    _quat_gt << so_msg.pose.pose.orientation.x, so_msg.pose.pose.orientation.y, so_msg.pose.pose.orientation.z, so_msg.pose.pose.orientation.w; 
    _frame_gt = so_msg.header.frame_id;
    _z_v.block<3,1>(0,0) = _pose_gt;
    _z_v.block<3,1>(3,0) = _vel_gt;
    _ii_odom_msg_received = true;
}   

void AKF_ros::ctrl_acc_cb( const mrs_msgs::EstimatorInput ref_acc ) {

    _cmd_acc << ref_acc.control_acceleration.x, ref_acc.control_acceleration.y, ref_acc.control_acceleration.z;
    _new_ctrl_acc = true;
    // std::cout<<_cmd_acc[0]<<", "<<_cmd_acc[1]<<", "<<_cmd_acc[2]<<"\n";
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

void AKF_ros::trace_cb( const std_msgs::Float32 tr_msg ) {
    _trace = tr_msg.data;
    _trace_received = true;
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
        if ( _takeoff_done ) {

            // u = _cmd_acc;

            if( _meas_l_ok[0] && !_q_change_ok[0] ) {
                Q(15,15) = Q(15,15)*_q_v_meas_l_ok[0];
                Q(18,18) = Q(18,18)*_q_v_meas_l_ok[0];
                _q_change_ok[0] = true;
                ROS_WARN("Good x LIO meas. Update offset VIO");
            }
            if( _meas_l_ok[1] && !_q_change_ok[1]) {
                Q(16,16) = Q(16,16)*_q_v_meas_l_ok[1];
                Q(19,19) = Q(19,19)*_q_v_meas_l_ok[1];
                _q_change_ok[1] = true;
                ROS_WARN("Good y LIO meas. Update offset VIO");
            }
            if( _meas_l_ok[2] && !_q_change_ok[2]) {
                Q(17,17) = Q(17,17)*_q_v_meas_l_ok[2];
                Q(20,20) = Q(20,20)*_q_v_meas_l_ok[2];
                _q_change_ok[2] = true;
                ROS_WARN("Good z LIO meas. Update offset VIO");
            }
            //--Predict
            x_p = A*x_u + B*_cmd_acc;
            P_p = A*P_u*A.transpose() + _Dt*Q;

            if( !_meas_l_ok[0] && !_rq_change_bad[0]){
                R_L(0,0) = R_L(0,0)*_r_l_bad[0];
                R_L(3,3) = R_L(3,3)*_r_l_bad[0];
                Q(9,9) = Q(9,9)*_q_l_meas_bad[0];
                Q(12,12) = Q(12,12)*_q_l_meas_bad[0];
                // ROS_INFO("Entro qui, X");
                _rq_change_bad[0] = true;
            }
            if( !_meas_l_ok[1] && !_rq_change_bad[1]){
                R_L(1,1) = R_L(1,1)*_r_l_bad[1];
                R_L(4,4) = R_L(4,4)*_r_l_bad[1];
                Q(10,10) = Q(10,10)*_q_l_meas_bad[1];
                Q(13,13) = Q(13,13)*_q_l_meas_bad[1];
                _rq_change_bad[1] = true;
                // ROS_INFO("Entro qui, Y");
            }
            if( !_meas_l_ok[2] && !_rq_change_bad[2] ){
                R_L(2,2) = R_L(2,2)*_r_l_bad[2];
                R_L(5,5) = R_L(5,5)*_r_l_bad[2];
                Q(11,11) = Q(11,11)*_q_l_meas_bad[0];
                Q(14,14) = Q(14,14)*_q_l_meas_bad[0];
                _rq_change_bad[2] = true;
                // ROS_INFO("Entro qui, Z");
            }

            //--Correct

            if( _lio_odom_msg_received /*&& _eig_received*/) {
                _eig_received=false;
                _lio_odom_msg_received=false;
                y = _z_l - H_L*x_p;
                K = P_p * H_L.transpose() * (H_L * P_p * H_L.transpose() + R_L).inverse();

                x_u = x_p + K*y;
                P_u = (Eigen::Matrix<double,21,21>::Identity() - K * H_L) * P_p;
                
            }
            else if( _ii_odom_msg_received ) {
                _ii_odom_msg_received = false;
                y = _z_v - H_V*x_p;
                K = P_p * H_V.transpose() * (H_V * P_p * H_V.transpose() + R_V).inverse();

                x_u = x_p + K*y;
                P_u = (Eigen::Matrix<double,21,21>::Identity() - K * H_V) * P_p;
            }

            odom_out_msg.pose.pose.position.x = x_u[0];
            odom_out_msg.pose.pose.position.y = x_u[1];
            odom_out_msg.pose.pose.position.z = x_u[2];
            odom_out_msg.twist.twist.linear.x = x_u[3];
            odom_out_msg.twist.twist.linear.y = x_u[4];
            odom_out_msg.twist.twist.linear.z = x_u[5];

            /*TODO: control after re-initialization*/

            _robot_est.publish(odom_out_msg);
        }

        

        r.sleep();
    }
}

void AKF_ros::fusion_loop_1d() {
    
    ros::Rate r( 80 );
    Eigen::Matrix<double,7, 7> A;
    Eigen::Matrix<double,7,1> B;
    Eigen::Matrix<double,2, 7> H_L;
    Eigen::Matrix<double,2,7> H_V;
    Eigen::Matrix<double,7,7> Q;
    Eigen::Matrix<double,2,2> R_l;
    Eigen::Matrix<double,2,2> R_v;
    // double r_l;
    // double r_v;
    double a = exp(-_Dt/_tau);
    A = sys_model::build_A_1d( a, _Dt );
    B = sys_model::build_B_1d( a );
    H_L = sys_model::build_Hl_1d();
    H_V = sys_model::build_Hv_1d();

    /*KF initialization*/
    Eigen::Matrix<double, 7, 7> P_p = Matrix<double, 7, 7>::Identity();
    Eigen::Matrix<double, 7, 7> P_u = Matrix<double, 7, 7>::Identity();
    Eigen::Matrix<double, 7, 1> x_p = Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> x_u = Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 2, 1> y = Matrix<double, 2, 1>::Zero();
    // double y = 0;
    Eigen::Matrix<double, 7, 2> K = Matrix<double, 7, 2>::Zero();
    Q = 0.1*Matrix<double, 7, 7>::Identity();
    R_l = 0.1*Matrix<double, 2, 2>::Identity();
    R_v = 0.1*Matrix<double, 2, 2>::Identity();
    nav_msgs::Odometry odom_out_msg;
    Eigen::Vector2d z_l_1d, z_v_1d;
    while( ros::ok() ) {

        if( _init_kf ) {
            _init_kf = false;
            x_p << _uav_pos[0], _uav_vel[0];
            x_u = x_p;
            ROS_INFO("Kalman filter's states initialized!");

        }

        if( _takeoff_done ) {
            if( _meas_l_ok[0] && !_q_change_ok[0] ) {
                Q(5,5) = Q(5,5)*_q_v_meas_l_ok[0];
                Q(6,6) = Q(6,6)*_q_v_meas_l_ok[0];
                _q_change_ok[0] = true;
                ROS_WARN("Good x LIO meas. Update offset VIO");
            }

            x_p = A*x_u + B*_cmd_acc[0];
            P_p = A*P_u*A.transpose() + _Dt*Q;

            if( !_meas_l_ok[0] && !_rq_change_bad[0]){
                R_l(0,0) = R_l(0,0)*_r_l_bad[0];
                Q(3,3) = Q(3,3)*_q_l_meas_bad[0];
                Q(4,4) = Q(4,4)*_q_l_meas_bad[0];
                ROS_WARN("Update cov for LIO X");
                Q(5,5) = Q(5,5)*1/_q_v_meas_l_ok[0];
                Q(6,6) = Q(6,6)*1/_q_v_meas_l_ok[0];
                _rq_change_bad[0] = true;
            }
            z_l_1d << _z_l(0,0), _z_l(3,0);
            z_v_1d << _z_v(0,0), _z_v(3,0);
            if( _lio_odom_msg_received /*&& _eig_received*/) {
                _eig_received=false;
                _lio_odom_msg_received=false;
                y = z_l_1d - H_L*x_p;
                // K = P_p * H_L.transpose() * 1/(H_L * P_p * H_L.transpose() + r_l);
                K = P_p * H_L.transpose() * (H_L * P_p * H_L.transpose() + R_l).inverse();

                x_u = x_p + K*y;
                P_u = (Eigen::Matrix<double, 7,7>::Identity() - K * H_L) * P_p;
                
            }
            else if( _ii_odom_msg_received ) {
                _ii_odom_msg_received = false;
                y = _z_v.block<2,1>(0,0) - H_V*x_p;
                // K = P_p * H_V.transpose() * 1/(H_V * P_p * H_V.transpose() + r_v);
                K = P_p * H_V.transpose() * (H_V * P_p * H_V.transpose() + R_v).inverse();

                x_u = x_p + K*y;
                P_u = (Eigen::Matrix<double,7,7>::Identity() - K * H_V) * P_p;
            }
        }
        odom_out_msg.pose.pose.position.x = x_u[0];

        odom_out_msg.twist.twist.linear.x = x_u[1];

        _robot_est.publish(odom_out_msg);
        if( _debug ) {
            for(int i=0; i<6; i++) {
                std::cout<<x_u(i)<<" ";
            }
            std::cout<<"\n";
            
        }
        r.sleep();
    }
}

void AKF_ros::fusion_loop_2d() {
    
    ros::Rate r( 80 );
    Eigen::Matrix<double,14, 14> A;
    Eigen::Matrix<double,14,2> B;
    Eigen::Matrix<double,4, 14> H_L;
    Eigen::Matrix<double,4,14> H_V;
    Eigen::Matrix<double,14,14> Q;
    Eigen::Matrix<double,4,4> R_l;
    Eigen::Matrix<double,4,4> R_v;
    // double r_l;
    // double r_v;
    double a = exp(-_Dt/_tau);
    A = sys_model::build_A_2d( a, _Dt );
    B = sys_model::build_B_2d( a );
    H_L = sys_model::build_Hl_2d();
    H_V = sys_model::build_Hv_2d();
    // for(int i=0; i<4; i++) {
    //     for(int j=0; j<14; j++) {
    //         std::cout<<H_V(i,j)<<" ";
    //     }
    //     std::cout<<"\n";
    // }
    // std::cout<<"Dimension H_v: rows="<<H_V.rows()<<" cols="<<H_V.cols()<<"\n";
    /*KF initialization*/
    Eigen::Matrix<double, 14, 14> P_p = Matrix<double, 14, 14>::Identity();
    Eigen::Matrix<double, 14, 14> P_u = Matrix<double, 14, 14>::Identity();
    Eigen::Matrix<double, 14, 1> x_p = Matrix<double, 14, 1>::Zero();
    Eigen::Matrix<double, 14, 1> x_u = Matrix<double, 14, 1>::Zero();
    Eigen::Matrix<double, 4, 1> y = Matrix<double, 4, 1>::Zero();
    // double y = 0;
    Eigen::Matrix<double, 14, 4> K = Matrix<double, 14, 4>::Zero();
    Q = 0.1*Matrix<double, 14, 14>::Identity();
    R_l = 0.1*Matrix<double, 4, 4>::Identity();
    R_v = 0.1*Matrix<double, 4, 4>::Identity();
    nav_msgs::Odometry odom_out_msg;
    Eigen::Vector2d u;
    Eigen::Vector4d z_l_2d, z_v_2d;
    std_msgs::Bool state_x;
    std_msgs::Bool state_y;
    bool if_first_update_x = true;
    bool if_first_update_y = true;
    while( ros::ok() ) {

        if( _init_kf ) {
            _init_kf = false;
            x_p << _uav_pos[0], _uav_pos[1], _uav_vel[0], _uav_vel[1];
            x_u = x_p;
            ROS_INFO("Kalman filter's states initialized!");

        }
        u << _cmd_acc[0], _cmd_acc[1];

        if( _takeoff_done ) {
            if( _meas_l_ok[0] && !_q_change_ok[0] ) {
                if( if_first_update_x ) {
                    Q(10,10) = Q(10,10)*_q_v_meas_l_ok[0];
                    Q(12,12) = Q(12,12)*_q_v_meas_l_ok[0];
                    if_first_update_x = false;   
                }
                else {
                    ROS_ERROR("I go back to the oldest covariances.");
                    Q(10,10) = Q(10,10)*_q_v_meas_l_ok[0];
                    Q(12,12) = Q(12,12)*_q_v_meas_l_ok[0]; 
                    R_l(0,0) = R_l(0,0)*1/_r_l_bad[0];
                    R_l(2,2) = R_l(2,2)*1/_r_l_bad[0];
                    Q(6,6) = Q(6,6)*1/_q_l_meas_bad[0];
                    Q(8,8) = Q(8,8)*1/_q_l_meas_bad[0];
                }

                _q_change_ok[0] = true;
                ROS_WARN("Good X LIO meas. Update offset VIO");
            }
            if( _meas_l_ok[1] && !_q_change_ok[1] ) {
                if( if_first_update_y ) {
                    Q(11,11) = Q(11,11)*_q_v_meas_l_ok[1];
                    Q(13,13) = Q(13,13)*_q_v_meas_l_ok[1];
                    if_first_update_y = false;

                }
                else {
                    R_l(1,1) = R_l(1,1)*1/_r_l_bad[1];
                    R_l(3,3) = R_l(3,3)*1/_r_l_bad[1];
                    Q(7,7) = Q(7,7)*1/_q_l_meas_bad[1];
                    Q(9,9) = Q(9,9)*1/_q_l_meas_bad[1];
                    Q(11,11) = Q(11,11)*_q_v_meas_l_ok[1];
                    Q(13,13) = Q(13,13)*_q_v_meas_l_ok[1];
                }

                _q_change_ok[1] = true;
                ROS_WARN("Good Y LIO meas. Update offset VIO");
            }

            x_p = A*x_u + B*u;
            P_p = A*P_u*A.transpose() + _Dt*Q;

            if( !_meas_l_ok[0] && !_rq_change_bad[0]){
                R_l(0,0) = R_l(0,0)*_r_l_bad[0];
                R_l(2,2) = R_l(2,2)*_r_l_bad[0];
                Q(6,6) = Q(6,6)*_q_l_meas_bad[0];
                Q(8,8) = Q(8,8)*_q_l_meas_bad[0];
                ROS_WARN("Update cov for LIO X");
                Q(10,10) = Q(10,10)*1/_q_v_meas_l_ok[0];
                Q(12,12) = Q(12,12)*1/_q_v_meas_l_ok[0];
                _rq_change_bad[0] = true;
            }
            if( !_meas_l_ok[1] && !_rq_change_bad[1]){
                R_l(1,1) = R_l(1,1)*_r_l_bad[1];
                R_l(3,3) = R_l(3,3)*_r_l_bad[1];
                Q(7,7) = Q(7,7)*_q_l_meas_bad[1];
                Q(9,9) = Q(9,9)*_q_l_meas_bad[1];
                ROS_WARN("Update cov for LIO Y");
                Q(11,11) = Q(11,11)*1/_q_v_meas_l_ok[1];
                Q(13,13) = Q(13,13)*1/_q_v_meas_l_ok[1];
                _rq_change_bad[1] = true;
            }
            z_l_2d << _z_l.block<2,1>(0,0), _z_l.block<2,1>(3,0);
            z_v_2d << _z_v.block<2,1>(0,0), _z_v.block<2,1>(3,0);
            // std::cout<<"z_l_2d = "<<z_l_2d[0]<<", "<<z_l_2d[1]<<"\n z_v_2d = "<<z_v_2d[0]<<", "<<z_v_2d[1]<<"\n\n";
            if( _lio_odom_msg_received /*&& _eig_received*/) {
                _eig_received=false;
                _lio_odom_msg_received=false;
                y = z_l_2d - H_L*x_p;
                K = P_p * H_L.transpose() * (H_L * P_p * H_L.transpose() + R_l).inverse();

                x_u = x_p + K*y;
                P_u = (Eigen::Matrix<double, 14,14>::Identity() - K * H_L) * P_p;
                
            }
            else if( _ii_odom_msg_received ) {
                _ii_odom_msg_received = false;
                y = z_v_2d - H_V*x_p;
                K = P_p * H_V.transpose() * (H_V * P_p * H_V.transpose() + R_v).inverse();

                x_u = x_p + K*y;
                P_u = (Eigen::Matrix<double,14,14>::Identity() - K * H_V) * P_p;
            }
        }
        odom_out_msg.pose.pose.position.x = x_u[0];
        odom_out_msg.pose.pose.position.y = x_u[1];
        odom_out_msg.twist.twist.linear.x = x_u[2];
        odom_out_msg.twist.twist.linear.y = x_u[3];

        state_x.data = _state_x;
        state_y.data = _state_y;


        _robot_est.publish(odom_out_msg);
        _filter_state_x.publish(state_x);
        _filter_state_y.publish(state_y);
        if( _debug ) {
            for(int i=0; i<6; i++) {
                std::cout<<x_u(i)<<" ";
            }
            std::cout<<"\n";
            
        }
        r.sleep();
    }
}

void AKF_ros::monitor_LIO() {
    ros::Rate r( 10 );

    static tf2_ros::StaticTransformBroadcaster new_init_broadcaster;
    geometry_msgs::TransformStamped new_init_tf;
    while( ros::ok() ) {

        if( _takeoff_done ) {

            if( sqrt(pow(_uav_pos[0]-_pose_gt[0],2) + pow(_uav_pos[1]-_pose_gt[1],2)) > _dist_th && !_dist_max ) {
                _dist_max = true;
                _t1 = ros::Time::now();
                // std::cout<<sqrt(pow(_uav_pos[0]-_pose_gt[0],2) + pow(_uav_pos[1]-_pose_gt[1],2))<<"\n";
                // ROS_INFO( "If for check position" );
            }
            if( sqrt(pow(_uav_pos[0]-_pose_gt[0],2) + pow(_uav_pos[1]-_pose_gt[1],2)) <= _dist_th ) {
                _dist_max = false;
                // ROS_INFO( "If to go back if the position is good" );
            }
            _t2 = ros::Time::now();
            if( (_t2 - _t1).toSec() > _time_th && _dist_max && !_killed ) {
                ROS_ERROR( "Killing the node %s!", _node_name.c_str() );

                std::string kill_cmd = "rosnode kill "+ _node_name;
                int kk = system( kill_cmd.c_str() );

                ros::Duration(3.0).sleep();

                ROS_INFO( "Node %s killed", _node_name.c_str() );
                _dist_max = true;
                _killed = true;
            }

            if( _killed && _dist_max ) {
                ROS_WARN( "Node re-initialization." );
                /**/
                new_init_tf.header.stamp = ros::Time::now();
                new_init_tf.header.frame_id = "uav1/local_origin";
                new_init_tf.child_frame_id = "new_lio_init";
                new_init_tf.transform.translation.x = _pose_gt[0];
                new_init_tf.transform.translation.y = _pose_gt[1];
                new_init_tf.transform.translation.z = _pose_gt[2];
                new_init_tf.transform.rotation.x = _quat_gt[0];
                new_init_tf.transform.rotation.y = _quat_gt[1];
                new_init_tf.transform.rotation.z = _quat_gt[2];
                new_init_tf.transform.rotation.w = _quat_gt[3];

                new_init_broadcaster.sendTransform(new_init_tf);

                /**/
                std::vector<double> vec = {1.0, 2.0, 3.0};
                std::string restart_cmd = "roslaunch point_lio mapping_ouster64_simu.launch new_frame:=new_lio_init new_x:="+std::to_string(_pose_gt[0])
                                            +" new_y:="+std::to_string(_pose_gt[1])+" new_z:="+std::to_string(_pose_gt[2]);
                int ll = system( restart_cmd.c_str() );
                _killed = false;
            }
        }

        
        r.sleep();
    }
}
void AKF_ros::run() {
    boost::thread check_drift_t( &AKF_ros::fusion_loop_2d, this );
    if( _do_reboot )
        boost::thread monitor_lio_t( &AKF_ros::monitor_LIO, this );
    ros::spin();
}

int main( int argc, char** argv ) {

    ros::init(argc, argv, "akf_ros");
    AKF_ros akf;
    akf.run();
    return 0;
}