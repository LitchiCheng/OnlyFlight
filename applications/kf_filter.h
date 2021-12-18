#pragma once
#include <math.h>
 
//#define RAD_TO_DEG (360/PI/2) // 弧度转角度的转换率
//#define DEG_TO_RAD (2*PI/360) // 角度转弧度的转换率
#define RAD_TO_DEG 57.295779513082320876798154814105  // 弧度转角度的转换率
#define DEG_TO_RAD 0.01745329251994329576923690768489 // 角度转弧度的转换率
 
// Comment out to restrict roll to ±90deg instead -
// please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define RESTRICT_PITCH
 
typedef struct
{
	float Q_angle;
	float Q_bias;
	float R_measure;
	float angle; // Reset the angle
	float bias; // Reset bias
	float P[2][2];
	// Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
	float rate;
} KalmanFilter_t;

typedef struct
{
	KalmanFilter_t *pKalmanX;
	KalmanFilter_t *pKalmanY;
	float gyroXangle, gyroYangle; // Angle calculate using the gyro only
	float compAngleX, compAngleY; // Calculated angle using a complementary filter
	float kalAngleX, kalAngleY;  // Calculated angle using a Kalman filter
} KalmanFilterSys_t;
 
// Need set starting angle
static KalmanFilterSys_t *Get_Kalman_Filter(float roll, float pitch)
{
	KalmanFilterSys_t *pSys = (KalmanFilterSys_t *)calloc(1, sizeof(KalmanFilterSys_t));
	pSys->pKalmanX = (KalmanFilter_t *)calloc(1, sizeof(KalmanFilter_t));
	pSys->pKalmanY = (KalmanFilter_t *)calloc(1, sizeof(KalmanFilter_t));
 
	/* We will set the variables like so, these can also be tuned by the user */
	pSys->pKalmanX->Q_angle = pSys->pKalmanY->Q_angle = 0.05f;
	pSys->pKalmanX->Q_bias = pSys->pKalmanY->Q_bias = 0.003f;
	pSys->pKalmanX->R_measure = pSys->pKalmanY->R_measure = 0.005f;
 
	pSys->pKalmanX->angle = roll; // Reset the angle
	pSys->pKalmanY->angle = pitch; // Reset bias
 
	// Since we assume that the bias is 0 and we know the starting angle (use setAngle),
	// the error covariance matrix is set like so -
	// see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
	pSys->pKalmanX->P[0][0] = 0.0f;
	pSys->pKalmanX->P[0][1] = 0.0f;
	pSys->pKalmanX->P[1][0] = 0.0f;
	pSys->pKalmanX->P[1][1] = 0.0f;
 
	pSys->pKalmanY->P[0][0] = 0.0f;
	pSys->pKalmanY->P[0][1] = 0.0f;
	pSys->pKalmanY->P[1][0] = 0.0f;
	pSys->pKalmanY->P[1][1] = 0.0f;
 
	pSys->gyroXangle = roll;
	pSys->gyroYangle = pitch;
 
	pSys->compAngleX = roll;
	pSys->compAngleY = pitch;
 
	return pSys;
}
 
// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
//eq. 25 and eq. 26
// atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
// It is then converted from radians to degrees
static void Accel_To_Angle(float *p_roll, float *p_pitch, float accX, float accY, float accZ)
{
#ifdef RESTRICT_PITCH // Eq. 25 and 26
	*p_pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
	*p_roll = atan2(accY, accZ) * RAD_TO_DEG;
#else     // Eq. 28 and 29
	*p_pitch = atan2(-accX, accZ) * RAD_TO_DEG;
	*p_roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
#endif
};
 
// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
static float Kalman_Filter_GetAngle(KalmanFilter_t *pSys, float newAngle, float newRate, float dt)
{
	// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
	// Modified by Kristian Lauszus
	// See my blog post for more information:  http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it
	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */
	pSys->rate = newRate - pSys->bias;
	pSys->angle += dt * pSys->rate;
 
	// Update estimation error covariance - Project the error covariance ahead
	/* Step 2 */
	pSys->P[0][0] += dt * (dt*pSys->P[1][1] - pSys->P[0][1] - pSys->P[1][0] + pSys->Q_angle);
	pSys->P[0][1] -= dt * pSys->P[1][1];
	pSys->P[1][0] -= dt * pSys->P[1][1];
	pSys->P[1][1] += pSys->Q_bias * dt;
 
	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	/* Step 4 */
	float S = pSys->P[0][0] + pSys->R_measure; // Estimate error
 
	/* Step 5 */
	float K[2]; // Kalman gain - This is a 2x1 vector
	K[0] = pSys->P[0][0] / S;
	K[1] = pSys->P[1][0] / S;
 
	// Calculate angle and bias - Update estimate with measurement zk (newAngle)
	/* Step 3 */
	float y = newAngle - pSys->angle; // Angle difference
 
	/* Step 6 */
	pSys->angle += K[0] * y;
	pSys->bias += K[1] * y;
 
	// Calculate estimation error covariance - Update the error covariance
	/* Step 7 */
	float P00_temp = pSys->P[0][0];
	float P01_temp = pSys->P[0][1];
 
	pSys->P[0][0] -= K[0] * P00_temp;
	pSys->P[0][1] -= K[0] * P01_temp;
	pSys->P[1][0] -= K[1] * P00_temp;
	pSys->P[1][1] -= K[1] * P01_temp;
 
	return pSys->angle;
};
 
static void Kalman_Fileter_SetAngle(KalmanFilterSys_t *pSys, float roll, float pitch,
	float gyroXrate, float gyroYrate, float dt)
{
#ifdef RESTRICT_PITCH
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((roll < -90 && pSys->kalAngleX > 90) || (roll > 90 && pSys->kalAngleX < -90)) {
		pSys->pKalmanX->angle = roll;
		pSys->compAngleX = roll;
		pSys->kalAngleX = roll;
		pSys->gyroXangle = roll;
	}
	else
		pSys->kalAngleX = Kalman_Filter_GetAngle(pSys->pKalmanX, roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
 
	if (abs(pSys->kalAngleX) > 90)
		gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
	pSys->kalAngleY = Kalman_Filter_GetAngle(pSys->pKalmanY, pitch, gyroYrate, dt);
#else
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((pitch < -90 && pSys->kalAngleY > 90) || (pitch > 90 && pSys->kalAngleY < -90)) {
		pSys->pKalmanY->angle = pitch;
		pSys->compAngleY = pitch;
		pSys->kalAngleY = pitch;
		pSys->gyroYangle = pitch;
	}
	else
		pSys->kalAngleY = Kalman_Filter_GetAngle(pSys->pKalmanY, pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
 
	if (abs(pSys->kalAngleY) > 90)
		gyroXrate = -(gyroXrate); // Invert rate, so it fits the restriced accelerometer reading
	pSys->kalAngleX = Kalman_Filter_GetAngle(pSys->pKalmanX, roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif
 
	pSys->gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
	pSys->gyroYangle += gyroYrate * dt;
 
	//pSys->gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
	//pSys->gyroYangle += kalmanY.getRate() * dt;
 
	// Calculate the angle using a Complimentary filter
	pSys->compAngleX = 0.93 * (pSys->compAngleX + gyroXrate * dt) + 0.07 * roll;
	pSys->compAngleY = 0.93 * (pSys->compAngleY + gyroYrate * dt) + 0.07 * pitch;
 
	// Reset the gyro angle when it has drifted too much
	if (pSys->gyroXangle < -180 || pSys->gyroXangle > 180)
		pSys->gyroXangle = pSys->kalAngleX;
	if (pSys->gyroYangle < -180 || pSys->gyroYangle > 180)
		pSys->gyroYangle = pSys->kalAngleY;
};