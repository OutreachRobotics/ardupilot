// del_ekf.cpp

/***************************************************************************
    Include headers :
***************************************************************************/

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "del_ekf.h"

/***************************************************************************
	Global variables declaration :
***************************************************************************/

	double i2[] = {1,0,
					0,1};
	double i4[] = {1,0,0,0,
					0,1,0,0,
					0,0,1,0,
					0,0,0,1};

	double proll[] = {1000,0,0,0,
					0,1000,0,0,
					0,0,1000,0,
					0,0,0,1000};
	double ppitch[] = {1000,0,0,0,
					0,1000,0,0,
					0,0,1000,0,
					0,0,0,1000};
	double pyaw[] = {1,0,
					0,1};

	double croll [] = {0,0,1,0};
	double cpitch [] = {0,0,1,0};
	double cyaw [] = {1,0};

	double R_value = 100;
	

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
	
	F_roll = Mat(4,4,i4);
	F_pitch = Mat(4,4,i4);
	F_yaw = Mat(2,2,i4);
	
	B_roll = Mat(4,1);
	B_pitch = Mat(4,1);
	B_yaw = Mat(2,1);
	
	Qe_roll = Mat(4,4,i4);
	Qe_pitch = Mat(4,4,i4);
	Qe_yaw = Mat(2,2,i2);
	
	Re_roll = R_value;
	Re_pitch = R_value;
	Re_yaw = R_value;
	
	C_roll = Mat(1,4,croll);
	C_pitch = Mat(1,4,cpitch);
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

	F_in_filt = F_in*0.1582 + last_F_in*0.1582 + last_F_in_filt*0.6835;
	last_F_in_filt = F_in_filt;
	last_F_in = F_in;

	return F_in_filt;
}


void DelEKF::linearDynamicsEstimation(Vector3f F_in, Vector3f measure)
{
	double F_in_array[] = {F_in.x, F_in.y, F_in.z};
	Mat F_in_mat = Mat(3,1,F_in_array);
	double measure_array[] = {measure.x, measure.y, measure.z};
	Mat measure_mat = Mat(3,1,measure_array); 

	Mat F_in_filt = commandLPF(F_in_mat);
	Mat measure_mat_corrected = gyro2statesDt(measure_mat);

	propagateStates(F_in_filt);
	wrapPropStates();
	propagateCovariance();
	stateCovarianceUpdate(measure_mat_corrected);
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

void DelEKF::stateCovarianceUpdate(Mat measure)
{
	Mat tempRoll = C_roll*P_roll_prop*(C_roll.t());
	Mat tempPitch = C_pitch*P_pitch_prop*(C_pitch.t());
	Mat tempYaw = C_yaw*P_yaw_prop*(C_yaw.t());

	Mat Ke_roll = P_roll_prop*(C_roll.t()) * (1/(tempRoll[0] + Re_roll));
	Mat Ke_pitch = P_pitch_prop*(C_pitch.t()) * (1/(tempPitch[0] + Re_pitch));
	Mat Ke_yaw = P_yaw_prop*(C_yaw.t()) * (1/(tempYaw[0] + Re_yaw));

	x_roll = x_roll_prop + Ke_roll*(measure[0]-x_roll_prop[PHI1DT_P]);
	x_pitch = x_pitch_prop + Ke_pitch*(measure[1]-x_pitch_prop[PHI2DT_P]);
	x_yaw = x_yaw_prop + Ke_yaw*(measure[2]-x_yaw_prop[PHI3DT_P]);

	P_roll = (I4x4-Ke_roll*C_roll) * P_roll_prop * (I4x4-Ke_roll*C_roll).t() + Ke_roll*Re_roll*Ke_roll.t();
	P_pitch = (I4x4-Ke_pitch*C_pitch) * P_pitch_prop * (I4x4-Ke_pitch*C_pitch).t() + Ke_pitch*Re_pitch*Ke_pitch.t();
	P_yaw = (I2x2-Ke_yaw*C_yaw) * P_yaw_prop * (I2x2-Ke_yaw*C_yaw).t() + Ke_yaw*Re_yaw*Ke_yaw.t();
}

void DelEKF::wrapPropStates()
{
	x_roll_prop[PHI1_C] += x_roll_prop[PHI1_C]<M_PI ? 2.0*M_PI : 0.0;
	x_roll_prop[PHI1_C] -= x_roll_prop[PHI1_C]>M_PI ? 2.0*M_PI : 0.0;
	x_roll_prop[PHI1_P] += x_roll_prop[PHI1_P]<M_PI ? 2.0*M_PI : 0.0;
	x_roll_prop[PHI1_P] -= x_roll_prop[PHI1_P]>M_PI ? 2.0*M_PI : 0.0;

	x_pitch_prop[PHI2_C] += x_pitch_prop[PHI2_C]<M_PI ? 2.0*M_PI : 0.0;
	x_pitch_prop[PHI2_C] -= x_pitch_prop[PHI2_C]>M_PI ? 2.0*M_PI : 0.0;
	x_pitch_prop[PHI2_P] += x_pitch_prop[PHI2_P]<M_PI ? 2.0*M_PI : 0.0;
	x_pitch_prop[PHI2_P] -= x_pitch_prop[PHI2_P]>M_PI ? 2.0*M_PI : 0.0;

	x_yaw_prop[PHI3_P] += x_yaw_prop[PHI3_P]<M_PI ? 2.0*M_PI : 0.0;
	x_yaw_prop[PHI3_P] -= x_yaw_prop[PHI3_P]>M_PI ? 2.0*M_PI : 0.0;
}

void DelEKF::wrapStates()
{
	x_roll[PHI1_C] += x_roll[PHI1_C]<M_PI ? 2.0*M_PI : 0.0;
	x_roll[PHI1_C] -= x_roll[PHI1_C]>M_PI ? 2.0*M_PI : 0.0;
	x_roll[PHI1_P] += x_roll[PHI1_P]<M_PI ? 2.0*M_PI : 0.0;
	x_roll[PHI1_P] -= x_roll[PHI1_P]>M_PI ? 2.0*M_PI : 0.0;

	x_pitch[PHI2_C] += x_pitch[PHI2_C]<M_PI ? 2.0*M_PI : 0.0;
	x_pitch[PHI2_C] -= x_pitch[PHI2_C]>M_PI ? 2.0*M_PI : 0.0;
	x_pitch[PHI2_P] += x_pitch[PHI2_P]<M_PI ? 2.0*M_PI : 0.0;
	x_pitch[PHI2_P] -= x_pitch[PHI2_P]>M_PI ? 2.0*M_PI : 0.0;

	x_yaw[PHI3_P] += x_yaw[PHI3_P]<M_PI ? 2.0*M_PI : 0.0;
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

Mat DelEKF::createCommandMat(Vector3f orientation)
{
	double command[] = {0.0000, orientation.x, 0.0000, orientation.x, 
						0.0000, orientation.y, 0.0000, orientation.y,
						0.0000, orientation.z};
	return Mat(10,1,command);
}

void DelEKF::update_R_coeff(float r_value)
{
	Re_pitch = r_value;
	Re_roll = r_value;
	Re_yaw = r_value;	
}

void DelEKF::update_length(float length)
{
	if(length<6.0f)
	{
		double froll [] = {0.999553,-0.357855,0.000439,0.351230,
0.002500,0.999553,0.000000,0.000439,
0.000826,0.660914,0.999174,-0.660916,
0.000001,0.000826,0.002499,0.999174};
		double fpitch [] = {0.999852,-0.118002,0.000139,0.111376,
0.002500,0.999852,0.000000,0.000139,
0.000262,0.209578,0.999738,-0.209578,
0.000000,0.000262,0.002500,0.999738};
		double fyaw [] = {1.000000,0.000000,
		0.002500,1.000000};
		
		double broll [] = {0.006903,0.000000,0.129868,0.000000};
		double bpitch [] = {-0.010867,0.000000,0.163306,0.000000};
		double byaw [] = {1.942125,0.000000};

		double k_lqr_array[] = {0.000000,0.000000,0.000000,0.000000,65.628005,128.624906,60.277595,241.652402,-0.000000,-0.000000,
-69.598605,-218.873665,-53.880482,-162.008795,-0.000000,-0.000000,-0.000000,-0.000000,-0.000000,-0.000000,
0.000000,0.000000,0.000000,0.000000,-0.000000,-0.000000,-0.000000,-0.000000,15.827118,64.039825};

		F_roll = Mat(4,4,froll);
		F_pitch = Mat(4,4,fpitch);
		F_yaw = Mat(2,2,fyaw);
		
		B_roll = Mat(4,1,broll);
		B_pitch = Mat(4,1,bpitch);
		B_yaw = Mat(2,1,byaw);

		k_lqr = Mat(3,10,k_lqr_array);
	}
	else if(length<12.0f)
	{
		double froll [] = {0.999793,-0.165521,0.000203,0.162457,
0.002500,0.999793,0.000000,0.000203,
0.000826,0.660967,0.999174,-0.660968,
0.000001,0.000826,0.002499,0.999174};
		double fpitch [] = {0.999932,-0.054577,0.000064,0.051513,
0.002500,0.999932,0.000000,0.000064,
0.000262,0.209583,0.999738,-0.209584,
0.000000,0.000262,0.002500,0.999738};
		double fyaw [] = {1.000000,0.000000,
0.002500,1.000000};
		
		double broll [] = {0.003193,0.000000,0.129868,0.000000};
		double bpitch [] = {-0.005026,0.000000,0.163306,0.000000};
		double byaw [] = {1.942125,0.000000};

		double k_lqr_array[] = {0.000000,0.000000,0.000000,0.000000,108.152407,193.429110,53.073776,185.337092,-0.000000,-0.000000,
-116.748013,-284.042192,-45.783551,-103.085713,0.000000,0.000000,0.000000,0.000000,-0.000000,0.000000,
0.000000,0.000000,0.000000,0.000000,-0.000000,-0.000000,-0.000000,-0.000000,15.827118,64.039825};
	
		F_roll = Mat(4,4,froll);
		F_pitch = Mat(4,4,fpitch);
		F_yaw = Mat(2,2,fyaw);
		
		B_roll = Mat(4,1,broll);
		B_pitch = Mat(4,1,bpitch);
		B_yaw = Mat(2,1,byaw);

		k_lqr = Mat(3,10,k_lqr_array);
	}
	else if(length<17.0f)
	{
		double froll [] = {0.999873,-0.101862,0.000125,0.099976,
0.002500,0.999873,0.000000,0.000125,
0.000826,0.660984,0.999174,-0.660985,
0.000001,0.000826,0.002499,0.999174};
		double fpitch [] = {0.999958,-0.033586,0.000040,0.031700,
0.002500,0.999958,0.000000,0.000040,
0.000262,0.209585,0.999738,-0.209585,
0.000000,0.000262,0.002500,0.999738};
		double fyaw [] = {1.000000,0.000000,
		0.002500,1.000000};
		
		double broll [] = {0.001965,0.000000,0.129868,0.000000};
		double bpitch [] = {-0.003093,0.000000,0.163306,0.000000};
		double byaw [] = {1.942125,0.000000};

		double k_lqr_array[] = {-0.000000,-0.000000,-0.000000,-0.000000,143.493861,224.817840,49.329376,158.164849,-0.000000,-0.000000,
-156.826423,-317.108500,-41.268237,-73.467628,0.000000,0.000000,0.000000,0.000000,-0.000000,-0.000000,
0.000000,0.000000,0.000000,0.000000,-0.000000,-0.000000,-0.000000,-0.000000,15.827118,64.039825};

		F_roll = Mat(4,4,froll);
		F_pitch = Mat(4,4,fpitch);
		F_yaw = Mat(2,2,fyaw);
		
		B_roll = Mat(4,1,broll);
		B_pitch = Mat(4,1,bpitch);
		B_yaw = Mat(2,1,byaw);

		k_lqr = Mat(3,10,k_lqr_array);
	}

}
