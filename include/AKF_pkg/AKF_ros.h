/*Standard C++ libraries*/
#include <iostream>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

/*ROS libraries*/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mrs_msgs/EstimatorInput.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>

#include <Eigen/Dense>
#include "boost/thread.hpp"
#include "sys_model.h"

class AKF_ros {

    public:

        AKF_ros();

        /*Callbacks*/
        void LIO_cb( const nav_msgs::Odometry );
        void ctrl_acc_cb( const mrs_msgs::EstimatorInput );
        void ctrl_ref_cb( const nav_msgs::Odometry );
        void eig_cb( const std_msgs::Float32MultiArray );
        void points_cb( const std_msgs::UInt16 );
        void trace_cb( const std_msgs::Float32 );
        /*Second source callback*/
        void second_odom_cb( const nav_msgs::Odometry );

        void AKF_creation( Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& H_l, Eigen::MatrixXd& H_v,
        Eigen::MatrixXd& Q, Eigen::MatrixXd& R_l, Eigen::MatrixXd& R_v );
        void fusion_loop();
        void fusion_loop_1d();

        void run();
    
    private:

        ros::NodeHandle _nh;

        ros::Subscriber _lio_odom_sub;
        ros::Subscriber _acc_ctrl_sub;
        ros::Subscriber _ref_ctrl_sub;
        ros::Subscriber _eig_sub;
        ros::Subscriber _trace_sub;
        ros::Subscriber _points_sub;
        ros::Subscriber _second_source_sub; 

        ros::Publisher _robot_est;
        
        Eigen::Vector3d _cmd_acc;
        Eigen::Vector3d _cmd_vel;
        Eigen::Vector3d _cmd_pos;
        Eigen::Vector3d _uav_pos;
        Eigen::Vector4d _uav_quat;
        Eigen::Vector3d _uav_vel;
        Eigen::Vector3d _uav_ang_vel;

        Eigen::Vector3d _pose_gt;
        Eigen::Vector3d _vel_gt;

        Eigen::Matrix<double,6,1> _z_l;
        Eigen::Matrix<double,6,1> _z_v;

        Eigen::Matrix<bool,3,1> _meas_l_ok;
        Eigen::Matrix<bool,3,1> _q_change_ok;
        Eigen::Matrix<bool,3,1> _rq_change_bad;

        Eigen::Vector3d _eig_xyz;
        int _points;
        double _trace;

        /*Flags*/
        bool _eig_received;
        bool _lio_odom_msg_received;
        bool _ii_odom_msg_received;
        bool _trace_received;
        bool _points_received;
        bool _new_ctrl_acc;
        bool _first_meas;
        bool _init_kf;
        bool _takeoff_done;

        /*Parameters*/
        int _states;
        int _inputs;
        double _tau;
        double _Dt;
        double _lambda_x;
        double _lambda_y;
        double _lambda_z;
        std::vector<double> _kx;
        std::vector<double> _ky;
        std::vector<double> _kz;
        std::vector<double> _epsilon;
        std::vector<double> _q_v_meas_l_ok;
        std::vector<double> _r_l_bad;
        std::vector<double> _q_l_meas_bad;
        bool _debug;

};