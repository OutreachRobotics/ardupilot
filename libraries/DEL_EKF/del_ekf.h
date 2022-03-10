
// del_ekf.h

#ifndef DEL_EKF_H
#define DEL_EKF_H

/***************************************************************************
    Include headers :
***************************************************************************/

#include "del_mat.h"

/***************************************************************************
    Macro :
***************************************************************************/

#define PHI1DT_C    0
#define PHI1_C      1
#define PHI1DT_P    2
#define PHI1_P      3
#define PHI2DT_C    0
#define PHI2_C      1
#define PHI2DT_P    2
#define PHI2_P      3
#define PHI3DT_P    0
#define PHI3_P      1

#define TS          (1/400.0f)


/***************************************************************************
	Enumerations :
***************************************************************************/


/***************************************************************************
	Class :
***************************************************************************/

class DelEKF
{
public:
    DelEKF();
    Mat gyro2statesDt(Mat gyro_in);
    Mat commandLPF(Mat F_in);
    void linearDynamicsEstimation(Vector3f F_in, Vector3f measure);
    void propagateStates(Mat F_in);
    void propagateCovariance();
    void stateCovarianceUpdate(Mat measure);
    void wrapPropStates();
    void wrapStates();
    Vector3f getPlatformOrientation();
    Mat getEKFStates();
    Mat getLQRgain();
    Mat createCommandMat(Vector3f orientation);
    void update_R_coeff(float r_value);
    void update_LQR_gain(float test);

private:
    Mat x_roll;
    Mat x_pitch;
    Mat x_yaw;
    Mat x_roll_prop;
    Mat x_pitch_prop;
    Mat x_yaw_prop;

    Mat P_roll;
    Mat P_pitch;
    Mat P_yaw;
    Mat P_roll_prop;
    Mat P_pitch_prop;
    Mat P_yaw_prop;

    Mat F_roll;
    Mat F_pitch;
    Mat F_yaw;
    Mat B_roll;
    Mat B_pitch;
    Mat B_yaw;

    Mat Qe_roll;
    Mat Qe_pitch;
    Mat Qe_yaw;
    double Re_roll;
    double Re_pitch;
    double Re_yaw;

    Mat C_roll;
    Mat C_pitch;
    Mat C_yaw;

    Mat I4x4;
    Mat I2x2;

    Mat last_F_in;
    Mat last_F_in_filt;

    Mat Q_trap_roll;
    Mat Q_trap_pitch;
    Mat Q_trap_yaw;

    Mat H_roll;
    Mat H_pitch;
    Mat H_yaw;

    Mat k_lqr;    
};

#endif