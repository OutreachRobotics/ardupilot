
// del_helper.h

#ifndef DEL_HELPER_H
#define DEL_HELPER_H

/***************************************************************************
    Include headers :
***************************************************************************/



/***************************************************************************
    Macro :
***************************************************************************/

// MODE ///////////////////////////////////////////////////////////////////
#define MAX_RC_INPUT                        100.0f
#define MID_RC_INPUT                        50.0f
#define MID_PPM_VALUE                       1500
#define LOW_PPM_VALUE                       1000

// RANGEFINDER ///////////////////////////////////////////////////////////
#define RANGEFINDER_OFFSET                  0.45f // To adjust

// RC CHANNEL ////////////////////////////////////////////////////////////
#define SEQUENCE_CHANNEL                    CH_12
#define STEALTH_CHANNEL                     CH_11
#define TAXI_CHANNEL                        CH_13
#define CALIB_CHANNEL                       CH_15
#define WRIST_CHANNEL                       CH_14

// ATTITUDE CONTROL //////////////////////////////////////////////////////
#define YAW_SENSITIVITY                     0.01f
#define ROLL_SENSITIVITY                    0.003f
#define PITCH_SENSITIVITY                   0.003f
#define PMAX_ACTUATOR_THRUST                17.4f
#define PMIN_ACTUATOR_THRUST                9.0f
#define RMAX_ACTUATOR_THRUST                8.6f
#define YMAX_ACTUATOR_MOMENT                ((RMAX_ACTUATOR_THRUST/2) * (LT_BACK_L+LT_FORWARD_L))
#define M_PLATFORM                          3.02f
#define MAX_PITCH                           0.4f // 22.9°
#define MAX_ROLL                            0.2f // 11.45°
#define MIN_PITCH                           0.0f
#define MIN_ROLL                            (-MAX_ROLL)
#define DEADBAND                            0.02f

// FILTER COEFFICIENTS //////////////////////////////////////////////////////
// Low pass filter coefficient fc = 15 Hz, Fs = 50 Hz
#define B1_LP                               0.5792
#define B0_LP                               0.5792
#define A0_LP                               0.1584
// Low pass filter coefficient fc = 25 Hz, Fs = 400 Hz
#define B1_DS                               0.1659
#define B0_DS                               0.1659
#define A0_DS                               -0.6682
// Low pass filter coefficient fc = 1 Hz, Fs = 50 Hz
#define B1_SP                               0.0592
#define B0_SP                               0.0592
#define A0_SP                               -0.8816
// Low pass filter coefficient fc = 1 Hz, Fs = 50 Hz
#define LPF_RANGE_B                         0.0592
#define LPF_RANGE_A                         0.8816

// PD GAIN ///////////////////////////////////////////////////////////////////
#define YAW_KP                              20.0f
#define YAW_KD                              12.0f
#define ROLL_KP                             28.0f
#define ROLL_KD                             46.0f
#define PITCH_KP                            50.0f
#define PITCH_KD                            50.0f

// MAIN MOTOR //////////////////////////////////////////////////////////////////////

// FORWARD THRUST
#define FT2PWM_COEF1_M                      -0.0916f 
#define FT2PWM_COEF1_B                      2.5263f 
#define FT2PWM_COEF2_M                      1.2881f 
#define FT2PWM_COEF2_B                      -38.0520f 
#define FT2PWM_COEF3_M                      -6.3686f 
#define FT2PWM_COEF3_B                      228.0061f 
#define FT2PWM_COEF4_M                      -1.7693f 
#define FT2PWM_COEF4_B                      1602.27f 
#define FT_MAX_PPM                          1950
#define FT_MIN_PPM                          1550
#define FT_OFF_PPM                          1500
#define FT_TF_B                             0.2564 
#define FT_TF_A                             0.4872 

// BACKWARD THRUST
#define BT2PWM_COEF1_M                      -0.6267f 
#define BT2PWM_COEF1_B                      17.3241f 
#define BT2PWM_COEF2_M                      -4.2450f 
#define BT2PWM_COEF2_B                      127.2179f 
#define BT2PWM_COEF3_M                      -10.2182f 
#define BT2PWM_COEF3_B                      384.7325f 
#define BT2PWM_COEF4_M                      2.3567f 
#define BT2PWM_COEF4_B                      1373.1471f 
#define BT_MAX_PPM                          1050
#define BT_MIN_PPM                          1450
#define BT_OFF_PPM                          1500

// LATERAL THRUST
#define LT_BACK_L                           0.245f
#define LT_FORWARD_L                        0.270f
#define LT2PWM_COEF1_M                      -0.0167f 
#define LT2PWM_COEF1_B                      6.3152f 
#define LT2PWM_COEF2_M                      0.3040f 
#define LT2PWM_COEF2_B                      -61.8064f 
#define LT2PWM_COEF3_M                      -1.4008f 
#define LT2PWM_COEF3_B                      242.7982f 
#define LT2PWM_COEF4_M                      1.5463f 
#define LT2PWM_COEF4_B                      1483.0381f 
#define LT_MAX_PPM                          1900
#define LT_MIN_PPM                          1100
#define LT_IDLE_PPM                         1530
#define LT_OFF_PPM                          1495
#define LT_TF_B                             0.1667 
#define LT_TF_A                             0.6667 

// EKF CONSTANT ////////////////////////////////////////////////////////////////
#define EYE_2                               {1,0,0,1}
#define EYE_4                               {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}
#define P_INI_2                             {1000,0,0,1000}
#define P_INI_4                             {1000,0,0,0,0,1000,0,0,0,0,1000,0,0,0,0,1000}
#define C_YAW                               {1,0}
#define C_ROLL                              {0,0,1,0,0,0,0,1}
#define C_PITCH                             {0,0,1,0,0,0,0,1}

// EKF TUNING ///////////////////////////////////////////////////////////////////
#define R_ROLL                              {0.1f,0,0,1e-4}
#define R_PITCH                             {0.1f,0,0,1e-4}
#define R_YAW_GYRO                          0.1f

#define TS                                  (1/400.0f)

#define K_LQR_TAXI                          {-0.0000,-0.0000,0.0000,0.0000,9.1023,10.6138,33.0339,-11.7028,0.0000,0.0000,-9.5826,-68.3837,-33.1381,68.7290,-0.0000,0.0000,-0.0000,-0.0000,0.0000,0.0000,0.000000,0.000000,0.000000,0.000000,-0.000000,-0.000000,-0.000000,-0.000000,15.827118,64.039825} 

#define F_ROLL_6M                           {0.999681,-0.255347,0.000312,0.249368,0.002500,0.999681,0.000000,0.000312,0.000775,0.619642,0.999225,-0.619644,0.000001,0.000775,0.002499,0.999225}
#define F_PITCH_6M                          {0.999896,-0.083469,0.000097,0.077489,0.002500,0.999896,0.000000,0.000097,0.000241,0.192549,0.999759,-0.192549,0.000000,0.000241,0.002500,0.999759}
#define F_YAW_6M                            {1.000000,0.000000,0.002500,1.000000}
#define B_ROLL_6M                           {0.040947,0.000000,0.095025,0.000000}
#define B_PITCH_6M                          {0.056299,0.000000,0.056878,0.000000}
#define B_YAW_6M                            {2.277904,0.000000}
#define K_LQR_6M                            {0.000000,0.000000,0.000000,0.000000,51.829804,100.146706,11.219632,10.379871,-0.000000,-0.000000,-39.849328,-77.764407,-22.606382,-31.579189,0.000000,0.000000,-0.000000,-0.000000,-0.000000,-0.000000,0.000000,0.000000,0.000000,-0.000000,-0.000000,0.000000,0.000000,-0.000000,20.312573,112.102555}

#define F_ROLL_10M                          {0.999681,-0.255347,0.000312,0.249368,0.002500,0.999681,0.000000,0.000312,0.000775,0.619642,0.999225,-0.619644,0.000001,0.000775,0.002499,0.999225}
#define F_PITCH_10M                         {0.999896,-0.083469,0.000097,0.077489,0.002500,0.999896,0.000000,0.000097,0.000241,0.192549,0.999759,-0.192549,0.000000,0.000241,0.002500,0.999759}
#define F_YAW_10M                           {1.000000,0.000000,0.002500,1.000000}
#define B_ROLL_10M                          {0.040947,0.000000,0.095025,0.000000}
#define B_PITCH_10M                         {0.056299,0.000000,0.056878,0.000000}
#define B_YAW_10M                           {2.277904,0.000000}
#define K_LQR_10M                           {-0.000000,-0.000000,-0.000000,0.000000,88.476972,336.706115,13.402805,-45.668263,-0.000000,-0.000000,-58.988180,-181.906262,-40.163426,-102.234798,0.000000,-0.000000,-0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,-0.000000,-0.000000,-0.000000,0.000000,0.000000,-0.000000,20.312573,112.102555}

#define F_ROLL_15M                          {0.999873,-0.101862,0.000125,0.099976,0.002500,0.999873,0.000000,0.000125,0.000826,0.660984,0.999174,-0.660985,0.000001,0.000826,0.002499,0.999174}
#define F_PITCH_15M                         {0.999958,-0.033586,0.000040,0.031700,0.002500,0.999958,0.000000,0.000040,0.000262,0.209585,0.999738,-0.209585,0.000000,0.000262,0.002500,0.999738}
#define F_YAW_15M                           {1.000000,0.000000,0.002500,1.000000}
#define B_ROLL_15M                          {0.001965,0.000000,0.129868,0.000000}
#define B_PITCH_15M                         {-0.003093,0.000000,0.163306,0.000000}
#define B_YAW_15M                           {1.942125,0.000000}
#define K_LQR_15M                           {-0.000000,-0.000000,-0.000000,-0.000000,143.493861,224.817840,49.329376,158.164849,-0.000000,-0.000000,-156.826423,-317.108500,-41.268237,-73.467628,0.000000,0.000000,0.000000,0.000000,-0.000000,-0.000000,0.000000,0.000000,0.000000,0.000000,-0.000000,-0.000000,-0.000000,-0.000000,15.827118,64.039825}

/***************************************************************************
	Enumerations :
***************************************************************************/



/***************************************************************************
	Class :
***************************************************************************/








#endif