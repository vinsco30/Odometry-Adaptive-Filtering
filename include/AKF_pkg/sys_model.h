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

    inline MatrixXd build_A_1d( float a, float Dt ) {


        
        Matrix<double, 7, 7> A = Matrix<double, 7,7>::Zero();
        A << 1.0, Dt, (Dt*Dt)/2, 0, 0, 0, 0,
            0, 1.0, Dt, 0, 0, 0, 0,
            0, 0, 1.0, a, 0, 0, 0,
            0, 0, 0, 1.0, 0, 0, 0,
            0, 0, 0, 0, 1.0, 0, 0,
            0, 0, 0, 0, 0, 1.0, 0,
            0, 0, 0, 0, 0, 0, 1.0;

            return A;
    }

    inline MatrixXd build_B_1d( float a ) {

        Matrix<double, 7, 1> B = Matrix<double, 7, 1>::Zero();

        B(2,0) = (1-a);

        return B;
    }

    inline MatrixXd build_Hl_1d() {

        Matrix<double, 2, 7> Hl = Matrix<double, 2, 7>::Zero();

        // Hl.block<6,6>(0,0) = Matrix<double, 6, 6>::Identity();
        // Hl.block<6,6>(9,0) = -1*Matrix<double, 6, 6>::Identity();

        // Hl.block<3,3>(3,3) = Matrix<double, 3, 3>::Identity();
        // Hl.block<3,3>(12,3) = 1*Matrix<double, 3, 3>::Identity();
        Hl << 1.0, 0, 0, -1.0, 0, 0, 0,
                0, 1.0, 0, 0, 1, 0, 0;

        return Hl;
    }

        inline MatrixXd build_Hv_1d() {

        Matrix<double, 2, 7> Hv = Matrix<double, 2, 7>::Zero();

        // Hv.block<6,6>(0,0) = Matrix<double, 6, 6>::Identity();
        // Hv.block<3,3>(9,0) = -1*Matrix<double, 3, 3>::Identity();

        // Hv.block<3,3>(3,3) = Matrix<double, 3, 3>::Identity();
        // Hv.block<3,3>(18,3) = 1*Matrix<double, 3, 3>::Identity();

        Hv << 1.0, 0, 0, 0, 0, -1.0, 0,
                0, 1.0, 0, 0, 0, 0, 1;

        return Hv;
    }

    inline MatrixXd build_A_2d( float a, float Dt ) {


        
        Matrix<double, 14, 14> A = Matrix<double, 14,14>::Zero();

        A <<1.0, 0.0, Dt, 0, (Dt*Dt)/2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 1.0, 0, Dt, 0, (Dt*Dt)/2, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 1.0, 0, Dt, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 1.0, 0, Dt, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, a, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, a, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0;

            return A;
    }

    inline MatrixXd build_B_2d( float a ) {

        Matrix<double, 14, 2> B = Matrix<double, 14, 2>::Zero();

        B(4,0) = (1-a);
        B(5,1) = (1-a);

        return B;
    }

    inline MatrixXd build_Hl_2d() {

        Matrix<double, 4, 14> Hl = Matrix<double, 4, 14>::Zero();

        // Hl.block<6,6>(0,0) = Matrix<double, 6, 6>::Identity();
        // Hl.block<6,6>(9,0) = -1*Matrix<double, 6, 6>::Identity();

        // Hl.block<3,3>(3,3) = Matrix<double, 3, 3>::Identity();
        // Hl.block<3,3>(12,3) = 1*Matrix<double, 3, 3>::Identity();
        Hl <<1.0, 0, 0, 0, 0, 0, -1.0, 0, 0, 0, 0, 0, 0, 0,
             0, 1.0, 0, 0, 0, 0, 0, -1.0, 0, 0, 0, 0, 0, 0,
             0, 0, 1.0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0,
             0, 0, 0, 1.0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0;

        return Hl;
    }

    inline MatrixXd build_Hv_2d() {

        Matrix<double, 4, 14> Hv = Matrix<double, 4, 14>::Zero();

        // Hl.block<6,6>(0,0) = Matrix<double, 6, 6>::Identity();
        // Hl.block<6,6>(9,0) = -1*Matrix<double, 6, 6>::Identity();

        // Hl.block<3,3>(3,3) = Matrix<double, 3, 3>::Identity();
        // Hl.block<3,3>(12,3) = 1*Matrix<double, 3, 3>::Identity();
        Hv <<1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1.0, 0, 0, 0,
             0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1.0, 0, 0,
             0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0,
             0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0;

        return Hv;
    }


}

#endif