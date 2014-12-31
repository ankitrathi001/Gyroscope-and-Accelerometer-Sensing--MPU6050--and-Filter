/*
 * Source : https://github.com/big5824/Quadrocopter/blob/master/declarations.h
 */

signed short ACCEL_XOUT = 0;
signed short ACCEL_YOUT = 0;
signed short ACCEL_ZOUT = 0;

float GYRO_XRATE = 0;
float GYRO_YRATE = 0;
float GYRO_ZRATE = 0;

int GYRO_XRATERAW = 0;
int GYRO_YRATERAW = 0;
int GYRO_ZRATERAW = 0;

unsigned char GYRO_XOUT_L = ' ';
unsigned char GYRO_XOUT_H = ' ';
unsigned char GYRO_YOUT_L = ' ';
unsigned char GYRO_YOUT_H = ' ';
unsigned char GYRO_ZOUT_L = ' ';
unsigned char GYRO_ZOUT_H = ' ';

signed short GYRO_XOUT = 0;
signed short GYRO_YOUT = 0;
signed short GYRO_ZOUT = 0;

unsigned char ACCEL_XOUT_L = ' ';
unsigned char ACCEL_XOUT_H = ' ';
unsigned char ACCEL_YOUT_L = ' ';
unsigned char ACCEL_YOUT_H = ' ';
unsigned char ACCEL_ZOUT_L = ' ';
unsigned char ACCEL_ZOUT_H = ' ';

signed long GYRO_XOUT_OFFSET_1000SUM = 0;
signed long GYRO_YOUT_OFFSET_1000SUM = 0;
signed long GYRO_ZOUT_OFFSET_1000SUM = 0;

float GYRO_XANGLE = 0;
float GYRO_YANGLE = 0;
float GYRO_ZANGLE = 0;

long GYRO_XANGLERAW = 0;
long GYRO_YANGLERAW = 0;
long GYRO_ZANGLERAW = 0;

float ACCEL_XANGLE = 0;
float ACCEL_YANGLE = 0;
float ACCEL_ZANGLE = 0;

float KALMAN_ANGLE_X = 0;
float KALMAN_ANGLE_Y = 0;
float KALMAN_ANGLE_Z = 0;

signed short GYRO_XOUT_OFFSET = 0;
signed short GYRO_YOUT_OFFSET = 0;
signed short GYRO_ZOUT_OFFSET = 0;

signed short ACCEL_XOUT_OFFSET = 0;
signed short ACCEL_YOUT_OFFSET = 0;
signed short ACCEL_ZOUT_OFFSET = 0;

/*
float COMPLEMENTARY_XANGLE = 0;
float COMPLEMENTARY_YANGLE = 0;
float TARGET_XANGLE = 0;
float TARGET_YANGLE = 0;
float TARGET_ZRATE = 0;
float PID_XOUTPUT = 0;
float PID_YOUTPUT = 0;
float PID_ZOUTPUT = 0;
float KP = 26.0; //25 27/6/12
float KI = 200.0; //85 5/6/12
float KD = 7; //7 27/6/12
float ZKP = 40.0; //40 8/6/12
float ZKD = 25.0; //25 8/6/12
float XERROR = 0;
float YERROR = 0;
float ZERROR = 0;
float throttle = 0;
float OC1_output = 0.0;
float OC2_output = 0.0;
float OC3_output = 0.0;
float OC4_output = 0.0;
int count = 0;
float XINTEGRAL = 0;
float YINTEGRAL = 0;
int throttle_input = 0;
int yaw_input = 0;
int pitch_input = 0;
int roll_input = 0;
*/

signed int accel_X[2] = {0};
signed int accel_Y[2] = {0};
signed int accel_Z[2] = {0};

signed int velocity_X[2] = {0};
signed int velocity_Y[2] = {0};
signed int velocity_Z[2] = {0};

signed int position_X[2] = {0};
signed int position_Y[2] = {0};
signed int position_Z[2] = {0};

signed int ACCEL_XOUT_OFFSET_1000SUM = 0;
signed int ACCEL_YOUT_OFFSET_1000SUM = 0;
signed int ACCEL_ZOUT_OFFSET_1000SUM = 0;

unsigned char countx = 0;
unsigned char county = 0;
unsigned char countz = 0;

float PRE_GYRO_XRATE = 0;
float PRE_GYRO_YRATE = 0;
float PRE_GYRO_ZRATE = 0;

signed int pre_position_X = 0;
signed int pre_position_Y = 0;
signed int pre_position_Z = 0;

typedef unsigned long long u64;
typedef unsigned long      u32;

float gyro_Rate_X[2] = {0};
float gyro_Rate_Y[2] = {0};
float gyro_Rate_Z[2] = {0};

float gyro_Angle_X[2] = {0};
float gyro_Angle_Y[2] = {0};
float gyro_Angle_Z[2] = {0};

float pre_gyro_Angle_X = 0;
float pre_gyro_Angle_Y = 0;
float pre_gyro_Angle_Z = 0;

float gyro_angle_temp_X = 0;
float gyro_angle_temp_Y = 0;
float gyro_angle_temp_Z = 0;
