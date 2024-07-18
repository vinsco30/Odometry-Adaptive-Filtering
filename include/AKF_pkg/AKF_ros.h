/*Standard C++ libraries*/
#include <iostream>
#include <cmath>
#include <fstream>
#include <string>

/*ROS libraries*/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mrs_msgs/EstimatorInput.h>

#include <Eigen/Core>
#include "boost/thread.hpp"
#include "sys_model.h"

class AKF_ros {

    public:

        AKF_ros();

        /*Callbacks*/
        void LIO_cb( const nav_msgs::Odometry );
        void ctrl_acc_cb( const mrs_msgs::EstimatorInput );
        void ctrl_ref_cb( const nav_msgs::Odometry );
        /*TODO: callbacks for degradancy metrics*/

        void AKF_creation();
        void fusion_loop();

        void run();
    
    private:

        ros::NodeHandle _nh;

        ros::Subscriber _lio_odom_sub;
        ros::Subscriber _acc_ctrl_sub;
        ros::Subscriber _ref_ctrl_sub;

        ros::Publisher _robot_est;
        
        Eigen::Vector3d _cmd_acc;
        Eigen::Vector3d _cmd_vel;
        Eigen::Vector3d _cmd_pos;
        Eigen::Vector3d _uav_pos;
        Eigen::Vector4d _uav_quat;
        Eigen::Vector3d _uav_vel;
        Eigen::Vector3d _uav_ang_vel;

        /*Parameters*/
        int _states;
        int _inputs;
        double _tau;
        double _Dt;

};