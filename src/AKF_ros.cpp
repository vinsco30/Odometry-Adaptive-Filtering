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
}

int main( int argc, char** argv ) {
    // ros::init(argc, argv, "akf_ros");

    return 0;
}