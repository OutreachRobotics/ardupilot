// del_ekf.cpp

/***************************************************************************
    Include headers :
***************************************************************************/

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "del_ekf.h"

/***************************************************************************
	Global variables declaration :
***************************************************************************/

	extern const AP_HAL::HAL& hal;

	double i2[] = EYE_2;
	double i4[] = EYE_4;

	double proll[] = P_INI_4;
	double ppitch[] = P_INI_4;
	double pyaw[] = P_INI_2;

	double croll [] = C_ROLL;
	double cpitch [] = C_PITCH;
	double cyaw [] = C_YAW;

	double R_roll [] = R_ROLL;
	double R_pitch [] = R_PITCH;

	double f_roll_array[] = F_ROLL;
	double b_roll_array[] = B_ROLL;
	double f_pitch_array[] = F_PITCH;
	double b_pitch_array[] = B_PITCH;
    double k_lqr_array[] = K_LQR;
	

/***************************************************************************
	Function definition :
***************************************************************************/

DelEKF::DelEKF()
{
	x_roll = Mat(4,1);
	x_pitch = Mat(4,1);
	x_yaw = Mat(2,1);

	x_roll_prop = Mat(4,1);
	x_pitch_prop = Mat(4,1);
	x_yaw_prop = Mat(2,1);
	
	P_roll = Mat(4,4);
	P_pitch = Mat(4,4);
	P_yaw = Mat(2,2);
	P_roll_prop = Mat(4,4);
	P_pitch_prop = Mat(4,4);
	P_yaw_prop = Mat(4,4);

	F_roll_multi = Mat(NUMBER_OF_LENGTH,16,f_roll_array);
	F_pitch_multi = Mat(NUMBER_OF_LENGTH,16,f_pitch_array);
	B_roll_multi = Mat(NUMBER_OF_LENGTH,4,b_roll_array);
	B_pitch_multi = Mat(NUMBER_OF_LENGTH,4,b_roll_array);
	
	F_roll = Mat(4,4,i4);
	F_pitch = Mat(4,4,i4);
	F_yaw = Mat(2,2,i4);
	
	B_roll = Mat(4,1);
	B_pitch = Mat(4,1);
	B_yaw = Mat(2,1);
	
	Qe_roll = Mat(4,4,i4);
	Qe_pitch = Mat(4,4,i4);
	Qe_yaw = Mat(2,2,i2);
	
	Re_roll = Mat(2,2,R_roll);
	Re_pitch = Mat(2,2,R_pitch);
	Re_yaw = R_YAW_GYRO;
	
	C_roll = Mat(2,4,croll);
	C_pitch = Mat(2,4,cpitch);
	C_yaw = Mat(1,2,cyaw);
	
	I4x4 = Mat(4,4,i4);
	I2x2 = Mat(2,2,i2);

	last_F_in = Mat(3,1);
    last_F_in_filt = Mat(3,1);

	Q_trap_roll = (F_roll*Qe_roll*(F_roll.t()) + Qe_roll) * (TS/2);
	Q_trap_pitch = (F_pitch*Qe_pitch*(F_pitch.t()) + Qe_pitch) * (TS/2);
	Q_trap_yaw = (F_yaw*Qe_yaw*(F_yaw.t()) + Qe_yaw) * (TS/2);

	H_roll = ((F_roll+I4x4)*B_roll)*(TS/2);
	H_pitch = ((F_pitch+I4x4)*B_pitch)*(TS/2);
	H_yaw = ((F_yaw+I2x2)*B_yaw)*(TS/2);

	k_lqr = Mat(3,10);
}

Mat DelEKF::gyro2statesDt(Mat gyro_in)
{
	double c1 = cos(x_roll[PHI1_P]);
	double c2 = cos(x_pitch[PHI2_P]);
	double s1 = sin(x_roll[PHI1_P]);
	double s2 = sin(x_pitch[PHI2_P]);
	
	Mat measure(3,1);
	measure[0] = (c2*gyro_in[0]*c1*c1 + s2*gyro_in[2]*c1 + c2*gyro_in[0]*s1*s1 + s2*gyro_in[1]*s1) / (c2*(c1*c1 + s1*s1));
	measure[1] = (c1*gyro_in[1] - s1*gyro_in[2]) / (c1*c1 + s1*s1);
	measure[2] = (c1*gyro_in[2] + s1*gyro_in[1]) / (c2*(c1*c1 + s1*s1));

	return measure;
}

Mat DelEKF::commandLPF(Mat F_in)
{
	Mat F_in_filt(3,1);

	F_in_filt[0] = F_in[0]*LT_TF_B + last_F_in[0]*LT_TF_B + last_F_in_filt[0]*LT_TF_A;
	F_in_filt[1] = F_in[1]*FT_TF_B + last_F_in[1]*FT_TF_B + last_F_in_filt[1]*FT_TF_A;
	F_in_filt[2] = F_in[2]*LT_TF_B + last_F_in[2]*LT_TF_B + last_F_in_filt[2]*LT_TF_A;
	last_F_in_filt = F_in_filt;
	last_F_in = F_in;

	return F_in_filt;
}


void DelEKF::linearDynamicsEstimation(Vector3f F_in, Vector3f gyro, Vector3f angle)
{
	double F_in_array[] = {F_in.x, F_in.y, F_in.z};
	Mat F_in_mat = Mat(3,1,F_in_array);
	double gyro_array[] = {gyro.x, gyro.y, gyro.z};
	Mat gyro_mat = Mat(3,1,gyro_array); 
	double angle_array[] = {angle.x, angle.y, angle.z};
	Mat angle_mat = Mat(3,1,angle_array); 

	Mat F_in_filt = commandLPF(F_in_mat);
	Mat gyro_mat_corrected = gyro2statesDt(gyro_mat);

	propagateStates(F_in_filt);
	wrapPropStates();
	propagateCovariance();
	stateCovarianceUpdate(gyro_mat_corrected, angle_mat);
	wrapStates();
}

void DelEKF::propagateStates(Mat F_in)
{
	H_roll = ((F_roll+I4x4)*B_roll)*(TS/2);
	H_pitch = ((F_pitch+I4x4)*B_pitch)*(TS/2);
	H_yaw = ((F_yaw+I2x2)*B_yaw)*(TS/2);

	x_roll_prop = F_roll*x_roll + H_roll*F_in[0];
	x_pitch_prop = F_pitch*x_pitch + H_pitch*F_in[1];
	x_yaw_prop = F_yaw*x_yaw + H_yaw*F_in[2];
}

void DelEKF::propagateCovariance()
{
	Q_trap_roll = (F_roll*Qe_roll*(F_roll.t()) + Qe_roll) * (TS/2);
	Q_trap_pitch = (F_pitch*Qe_pitch*(F_pitch.t()) + Qe_pitch) * (TS/2);
	Q_trap_yaw = (F_yaw*Qe_yaw*(F_yaw.t()) + Qe_yaw) * (TS/2);

	P_roll_prop = F_roll*P_roll*(F_roll.t()) + Q_trap_roll;
	P_pitch_prop = F_pitch*P_pitch*(F_pitch.t()) + Q_trap_pitch;
	P_yaw_prop = F_yaw*P_yaw*(F_yaw.t()) + Q_trap_yaw;
}

void DelEKF::stateCovarianceUpdate(Mat gyro, Mat angle)
{
	double roll_measure_array[] = {gyro[0],angle[0]};
	double pitch_measure_array[] = {gyro[1],angle[1]};
	double yaw_measure = gyro[2];

	Mat roll_measure = Mat(2,1,roll_measure_array);
	Mat pitch_measure = Mat(2,1,pitch_measure_array);

	Mat roll_calc = C_roll*x_roll_prop;
	Mat pitch_calc = C_pitch*x_pitch_prop;

	Mat tempRoll = (C_roll*P_roll_prop*(C_roll.t())) + Re_roll;
	Mat tempPitch = (C_pitch*P_pitch_prop*(C_pitch.t())) + Re_pitch;
	Mat tempYaw = (C_yaw*P_yaw_prop*(C_yaw.t()));

	Mat Ke_roll = P_roll_prop*(C_roll.t()) * (tempRoll.inv());
	Mat Ke_pitch = P_pitch_prop*(C_pitch.t()) * (tempPitch.inv());
	Mat Ke_yaw = P_yaw_prop*(C_yaw.t()) * (1/(tempYaw[0] + Re_yaw));

	x_roll = x_roll_prop + Ke_roll*(roll_measure-roll_calc);
	x_pitch = x_pitch_prop + Ke_pitch*(pitch_measure-pitch_calc);
	x_yaw = x_yaw_prop + Ke_yaw*(yaw_measure-x_yaw_prop[PHI3DT_P]);

	P_roll = (I4x4-Ke_roll*C_roll) * P_roll_prop * (I4x4-Ke_roll*C_roll).t() + Ke_roll*Re_roll*Ke_roll.t();
	P_pitch = (I4x4-Ke_pitch*C_pitch) * P_pitch_prop * (I4x4-Ke_pitch*C_pitch).t() + Ke_pitch*Re_pitch*Ke_pitch.t();
	P_yaw = (I2x2-Ke_yaw*C_yaw) * P_yaw_prop * (I2x2-Ke_yaw*C_yaw).t() + Ke_yaw*Re_yaw*Ke_yaw.t();
}

void DelEKF::wrapPropStates()
{
	x_roll_prop[PHI1_C] += x_roll_prop[PHI1_C]<-M_PI ? 2.0*M_PI : 0.0;
	x_roll_prop[PHI1_C] -= x_roll_prop[PHI1_C]>M_PI ? 2.0*M_PI : 0.0;
	x_roll_prop[PHI1_P] += x_roll_prop[PHI1_P]<-M_PI ? 2.0*M_PI : 0.0;
	x_roll_prop[PHI1_P] -= x_roll_prop[PHI1_P]>M_PI ? 2.0*M_PI : 0.0;

	x_pitch_prop[PHI2_C] += x_pitch_prop[PHI2_C]<-M_PI ? 2.0*M_PI : 0.0;
	x_pitch_prop[PHI2_C] -= x_pitch_prop[PHI2_C]>M_PI ? 2.0*M_PI : 0.0;
	x_pitch_prop[PHI2_P] += x_pitch_prop[PHI2_P]<-M_PI ? 2.0*M_PI : 0.0;
	x_pitch_prop[PHI2_P] -= x_pitch_prop[PHI2_P]>M_PI ? 2.0*M_PI : 0.0;

	x_yaw_prop[PHI3_P] += x_yaw_prop[PHI3_P]<-M_PI ? 2.0*M_PI : 0.0;
	x_yaw_prop[PHI3_P] -= x_yaw_prop[PHI3_P]>M_PI ? 2.0*M_PI : 0.0;
}

void DelEKF::wrapStates()
{
	x_roll[PHI1_C] += x_roll[PHI1_C]<-M_PI ? 2.0*M_PI : 0.0;
	x_roll[PHI1_C] -= x_roll[PHI1_C]>M_PI ? 2.0*M_PI : 0.0;
	x_roll[PHI1_P] += x_roll[PHI1_P]<-M_PI ? 2.0*M_PI : 0.0;
	x_roll[PHI1_P] -= x_roll[PHI1_P]>M_PI ? 2.0*M_PI : 0.0;

	x_pitch[PHI2_C] += x_pitch[PHI2_C]<-M_PI ? 2.0*M_PI : 0.0;
	x_pitch[PHI2_C] -= x_pitch[PHI2_C]>M_PI ? 2.0*M_PI : 0.0;
	x_pitch[PHI2_P] += x_pitch[PHI2_P]<-M_PI ? 2.0*M_PI : 0.0;
	x_pitch[PHI2_P] -= x_pitch[PHI2_P]>M_PI ? 2.0*M_PI : 0.0;

	x_yaw[PHI3_P] += x_yaw[PHI3_P]<-M_PI ? 2.0*M_PI : 0.0;
	x_yaw[PHI3_P] -= x_yaw[PHI3_P]>M_PI ? 2.0*M_PI : 0.0;
}

Vector3f DelEKF::getPlatformOrientation()
{
	return Vector3f(x_roll[PHI1_P], x_pitch[PHI2_P], x_yaw[PHI3_P]);
}

Mat DelEKF::getEKFStates()
{
	double states[] = {x_roll[PHI1DT_C], x_roll[PHI1_C], x_roll[PHI1DT_P], x_roll[PHI1_P], 
						x_pitch[PHI2DT_C], x_pitch[PHI2_C], x_pitch[PHI2DT_P], x_pitch[PHI2_P], 
						x_yaw[PHI3DT_P], x_yaw[PHI3_P]};
	return Mat(10,1,states);
}

Mat DelEKF::getLQRgain()
{
	return k_lqr;
}

Mat DelEKF::getLQRgain_taxi()
{
	double k_lqr_taxi[] = K_LQR_TAXI;
	return Mat(3,10,k_lqr_taxi);
}

Mat DelEKF::createCommandMat(Vector3f orientation)
{
	double command[] = {0.0000, orientation.x, 0.0000, orientation.x, 
						0.0000, orientation.y, 0.0000, orientation.y,
						0.0000, orientation.z};
	return Mat(10,1,command);
}

void DelEKF::update_R_coeff(float r_value)
{
	double roll_R_array [] = {0.1f,0,0,r_value};
	double pitch_R_array [] = {0.1f,0,0,r_value};
	Re_roll = Mat(2,2,roll_R_array);
	Re_pitch = Mat(2,2,pitch_R_array);
	Re_yaw = 0.1f;		
}

void DelEKF::update_length(float length)
{
	double fyaw [] = F_YAW;
	double byaw [] = B_YAW;
	F_yaw = Mat(2,2,fyaw);		
	B_yaw = Mat(2,1,byaw);

	if(round(length)>=MIN_LENGTH && round(length)<=MAX_LENGTH)
	{
		F_roll = Mat(4,4,F_roll_multi.getLength(uint8_t(round(length))));
		F_pitch = Mat(4,4,F_pitch_multi.getLength(uint8_t(round(length))));
		B_roll = Mat(4,1,B_roll_multi.getLength(uint8_t(round(length))));
		B_pitch = Mat(4,1,B_pitch_multi.getLength(uint8_t(round(length))));
		k_lqr = Mat(3,10,k_lqr_multi.getLength(uint8_t(round(length))));
	}
	else
	{
		F_roll = Mat(4,4,F_roll_multi.getLength(uint8_t(LENGTH_DEFAULT_VALUE)));
		F_pitch = Mat(4,4,F_pitch_multi.getLength(uint8_t(LENGTH_DEFAULT_VALUE)));
		B_roll = Mat(4,1,B_roll_multi.getLength(uint8_t(LENGTH_DEFAULT_VALUE)));
		B_pitch = Mat(4,1,B_pitch_multi.getLength(uint8_t(LENGTH_DEFAULT_VALUE)));
		k_lqr = Mat(3,10,k_lqr_multi.getLength(uint8_t(LENGTH_DEFAULT_VALUE)));
	}
}
