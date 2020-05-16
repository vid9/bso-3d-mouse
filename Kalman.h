/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

 // Original library in C++. Ported to C, to be used by common MCU.

#ifndef _Kalman_h_
#define _Kalman_h_

typedef struct kalman{
float Q_angle; // Process noise variance for the accelerometer
float Q_bias; // Process noise variance for the gyro bias
float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector

float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
} kalman;

void kalman_init(kalman * p_kalman) {

    /* We will set the variables like so, these can also be tuned by the user */

    p_kalman->Q_angle = 0.001f;
    p_kalman->Q_bias = 0.003f;
    p_kalman->R_measure = 0.03f;

    p_kalman->angle = 0.0f; // Reset the angle
    p_kalman->bias = 0.0f; // Reset bias

    p_kalman->P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    p_kalman->P[0][1] = 0.0f;
    p_kalman->P[1][0] = 0.0f;
    p_kalman->P[1][1] = 0.0f;
}


float kalman_get_angle(kalman * p_kalman, float newAngle, float newRate, float dt){
    p_kalman->rate = newRate - p_kalman->bias;
    p_kalman->angle += dt * p_kalman->rate;

    // Update estimation error covariance - Project the error covariance ahead
    
    p_kalman->P[0][0] += dt * (dt*p_kalman->P[1][1] - p_kalman->P[0][1] - p_kalman->P[1][0] + p_kalman->Q_angle);
    p_kalman->P[0][1] -= dt * p_kalman->P[1][1];
    p_kalman->P[1][0] -= dt * p_kalman->P[1][1];
    p_kalman->P[1][1] += p_kalman->Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    
    float S = p_kalman->P[0][0] + p_kalman->R_measure; // Estimate error
    
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = p_kalman->P[0][0] / S;
    K[1] = p_kalman->P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    
    float y = newAngle - p_kalman->angle; // Angle difference
    
    p_kalman->angle += K[0] * y;
    p_kalman->bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    
    float P00_temp = p_kalman->P[0][0];
    float P01_temp = p_kalman->P[0][1];

    p_kalman->P[0][0] -= K[0] * P00_temp;
    p_kalman->P[0][1] -= K[0] * P01_temp;
    p_kalman->P[1][0] -= K[1] * P00_temp;
    p_kalman->P[1][1] -= K[1] * P01_temp;

    return p_kalman->angle;
}

void setAngle(kalman * p_kalman , float angle) {
	p_kalman->angle=angle;
}

/*
void kalman_init(kalman * p_kalman);
float kalman_get_angle(kalman * p_kalman,float newAngle, float newRate, float dt);
void setAngle(kalman * p_kalman, float angle);

typedef struct kalmanX {
float Q_angle; // Process noise variance for the accelerometer
float Q_bias; // Process noise variance for the gyro bias
float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector

float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
} kalmanX;

typedef struct kalmanY {
float Q_angle; // Process noise variance for the accelerometer
float Q_bias; // Process noise variance for the gyro bias
float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector

float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
} kalmanY;


void kalman_initX(kalmanX * p_kalman);
float kalman_get_angleX(kalmanX * p_kalman,float newAngle, float newRate, float dt);
void setAngleX(kalmanX * p_kalman, float angle);

void kalman_initY(kalmanY * p_kalman);
float kalman_get_angleY(kalmanY * p_kalman,float newAngle, float newRate, float dt);
void setAngleY(kalmanY * p_kalman, float angle);
*/
#endif