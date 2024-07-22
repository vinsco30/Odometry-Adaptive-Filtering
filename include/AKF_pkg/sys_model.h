#ifndef _sysmodel_hpp
#define _sysmodel_hpp

#include <Eigen/Core>

using namespace Eigen;


namespace sys_model{

    inline MatrixXd build_A( float a, float Dt ) {
        int n_states = 21;
        MatrixXd A;
        A.resize(21,21); 
        A = Matrix<double, 21, 21>::Zero();

        A.block<3,3>(0,0) = Matrix<double, 3, 3>::Identity();
        A.block<3,3>(0,3) = Dt*Matrix<double, 3, 3>::Identity();
        A.block<3,3>(0,6) = ((Dt*Dt)/2)*Matrix<double, 3, 3>::Identity();

        A.block<3,3>(3,3) = Matrix<double, 3, 3>::Identity();
        A.block<3,3>(3,6) = Dt*Matrix<double, 3, 3>::Identity();

        A.block<3,3>(6,6) = a*Matrix<double, 3, 3>::Identity();

        A.block<12,12>(9,9) = Matrix<double, 12, 12>::Identity();

        return A;
    }

    inline MatrixXd build_B( float a ) {
        int n_states = 21;
        int n_inputs = 3;
        Matrix<double, 21, 3> B = Matrix<double, 21, 3>::Zero();

        B.block<3,3>(6,0) = (1-a)*Matrix<double, 3, 3>::Identity();

        return B;
    }

    inline MatrixXd build_Hl() {
        int n_states = 21;
        int n_vstates = 6;
        Matrix<double, 6, 21> Hl = Matrix<double, 6, 21>::Zero();

        // Hl.block<6,6>(0,0) = Matrix<double, 6, 6>::Identity();
        // Hl.block<6,6>(9,0) = -1*Matrix<double, 6, 6>::Identity();

        // Hl.block<3,3>(3,3) = Matrix<double, 3, 3>::Identity();
        // Hl.block<3,3>(12,3) = 1*Matrix<double, 3, 3>::Identity();
        Hl << 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 
                0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;

        return Hl;
    }

        inline MatrixXd build_Hv() {
        int n_states = 21;
        int n_vstates = 6;
        Matrix<double, 6, 21> Hv = Matrix<double, 6, 21>::Zero();

        // Hv.block<6,6>(0,0) = Matrix<double, 6, 6>::Identity();
        // Hv.block<3,3>(9,0) = -1*Matrix<double, 3, 3>::Identity();

        // Hv.block<3,3>(3,3) = Matrix<double, 3, 3>::Identity();
        // Hv.block<3,3>(18,3) = 1*Matrix<double, 3, 3>::Identity();

        Hv << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0,
                0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 
                0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

        return Hv;
    }

    inline MatrixXd build_Q() {
        Matrix<double, 21, 21> Q = 0.1*Matrix<double, 21, 21>::Identity();
        return Q;
    }

    inline MatrixXd build_R() {
        Matrix<double, 6, 6> R = 0.1*Matrix<double, 6, 6>::Identity();
        return R;
    }


}

#endif