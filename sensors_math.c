//#include <stdio.h>
#include <math.h>
#include <stdlib.h>
//#include <sys/time.h>
//#include <stdbool.h>
//#include <stdint.h>
//#include <string.h>
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "i2c/i2c.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Kalman.h"

const double RAD_TO_DEG = 57.29577951;

#define PCF_ADDRESS	0x38
#define MPU_ADDRESS	0x68
#define BUS_I2C		0
#define SCL 14
#define SDA 12

//					mask	returned value
#define button1		0x20	// 0b ??0? ????
#define button2		0x10	// 0b ???0 ????
#define button3		0x80	// 0b 0??? ????
#define button4		0x40	// 0b ?0?? ????
#define clr_btn		0xf0

#define led1 		0xfe	// 0b ???? ???0
#define led2 		0xfd	// 0b ???? ??0?
#define led3 		0xfb	// 0b ???? ?0??
#define led4 		0xf7	// 0b ???? 0???
#define leds_off	0xff

#define gpio_wemos_led	2
#define RESTRICT_PITCH

kalman *kalmanX, *kalmanY;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

double A;
double gyroXAngle, gyroYAngle;
double compXAngle, compYAngle;
double kalXAngle, kalYAngle;

typedef enum {
	MPU9250_ACCEL_X = 0x3b,
	MPU9250_ACCEL_Y = 0x3d,
	MPU9250_ACCEL_Z = 0x3f,
	MPU9250_TEMP = 0x41,
	MPU9250_GYRO_X = 0x43,
	MPU9250_GYRO_Y = 0x45,
	MPU9250_GYRO_Z = 0x47
} mpu9250_quantity;

uint32_t timer;

const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

struct timeval tval_before, tval_after, tval_result;

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
// write byte to PCF on I2C bus
void write_byte_pcf(uint8_t data) {
	i2c_slave_write(BUS_I2C, PCF_ADDRESS, NULL, &data, 1);
}

// read byte from PCF on I2C bus
uint8_t read_byte_pcf() {
	uint8_t data;
	i2c_slave_read(BUS_I2C, PCF_ADDRESS, NULL, &data, 1);
	return data;
}

// check for pressed buttons
void pcf_task(void *pvParameters) {

	uint8_t pcf_byte;

	// turn off all leds
	write_byte_pcf(leds_off);

	while (1) {

		pcf_byte = read_byte_pcf();

		// button 1 is pressed
		if ((pcf_byte & button1) == 0) {
			// clear buttons states and toggle led 1
			write_byte_pcf((pcf_byte ^ ~led1) | clr_btn);


			// button 2 is pressed
		} else if ((pcf_byte & button2) == 0) {
			// clear buttons states and turn on led 2
			write_byte_pcf((pcf_byte & led2) | clr_btn);


			// button 3 is pressed
		} else if ((pcf_byte & button3) == 0) {
			// clear buttons states and turn off led 3
			write_byte_pcf((pcf_byte | ~led3) | clr_btn);

			// button 4 is pressed
		} else if ((pcf_byte & button4) == 0) {
			// blink led 4
			write_byte_pcf(pcf_byte ^ ~led4);
		}

		// check again after 200 ms
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
*/
// read 2 bytes from MPU-9250 on I2C bus
uint16_t read_bytes_mpu(mpu9250_quantity quantity) {

	// high and low byte of quantity
	uint8_t data_high, data_low;
	uint8_t register_address = (uint8_t) quantity;

	i2c_slave_read(BUS_I2C, MPU_ADDRESS, &register_address, &data_high, 1);
	register_address++;
	i2c_slave_read(BUS_I2C, MPU_ADDRESS, &register_address, &data_low, 1);

	return (data_high << 8) + data_low;
}

// check MPU-9250 sensor values
void mpu_task(void *pvParameters) {
	uint16_t threshold = 10000;
	while (1) {
		// turn off Wemos led
		gpio_write(gpio_wemos_led, 1);
		printf("Accel_z: %d \n", read_bytes_mpu(MPU9250_ACCEL_Z));
		printf("Accel_x: %d \n", read_bytes_mpu(MPU9250_ACCEL_X));
		printf("Accel_y: %d \n", read_bytes_mpu(MPU9250_ACCEL_Y));
		printf("Gyro_z: %d \n", read_bytes_mpu(MPU9250_GYRO_Z));
		printf("Gyro_x: %d \n", read_bytes_mpu(MPU9250_GYRO_X));
		printf("Gyro_y: %d \n", read_bytes_mpu(MPU9250_GYRO_Y));

		if (read_bytes_mpu(MPU9250_ACCEL_Z) < threshold)
			// turn on Wemos led

			gpio_write(gpio_wemos_led, 0);

		// check again after 100 ms
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

void loop_task(void* pvParametrs) {
	while(1) {
		accX = read_bytes_mpu(MPU9250_ACCEL_X);
		accY = read_bytes_mpu(MPU9250_ACCEL_Y);
		accZ = read_bytes_mpu(MPU9250_ACCEL_Z);
		gyroX = read_bytes_mpu(MPU9250_GYRO_X);
		gyroY = read_bytes_mpu(MPU9250_GYRO_Y);
		gyroZ = read_bytes_mpu(MPU9250_GYRO_Z);

		gettimeofday(&tval_after, NULL);
		timersub(&tval_after, &tval_before, &tval_result);
		double dt = (double)tval_result.tv_usec; // Calculate delta time
		gettimeofday(&tval_before, NULL);

		#ifdef RESTRICT_PITCH // Eq. 25 and 26
			double roll = atan2(accY, accZ) * RAD_TO_DEG;
			double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
		#else // Eq. 28 and 29
			double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
			double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
		#endif

		double gyroXRate = gyroX / 131.0; // Convert to deg/s
		double gyroYRate = gyroY / 131.0; // Convert to deg/s

		#ifdef RESTRICT_PITCH
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
			if ((roll < -90 && kalXAngle > 90) || (roll > 90 && kalXAngle < -90)) {
				//setAngle(kalman, roll);
				setAngle(kalmanX, roll); 
				compXAngle = roll;
				kalXAngle = roll;
				gyroXAngle = roll;
			} else
				kalXAngle = kalman_get_angle(kalmanX, roll, gyroXRate, dt); // Calculate the angle using a Kalman filter  
				//kalXAngle = kalman_get_angle(kalman, roll, gyroXRate, dt); // Calculate the angle using a Kalman filter 
			if (abs(kalXAngle) > 90)
				gyroYRate = -gyroYRate; // Invert rate, so it fits the restriced accelerometer reading
			kalYAngle = kalman_get_angle(kalmanY, pitch, gyroYRate, dt); 
			//kalYAngle = kalman_get_angle(kalman, pitch, gyroYRate, dt); 
		#else
			// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
			if ((pitch < -90 && kalYAngle > 90) || (pitch > 90 && kalYAngle < -90)) {
				//setAngle(kalman, pitch); 
				setAngle(kalmanY, pitch); 
				compYAngle = pitch;
				kalYAngle = pitch;
				A = pitch;
			} else
				kalYAngle = kalman_get_angle(kalmanY, pitch, gyroYRate, dt); // Calculate the angle using a Kalman filter   
				//kalYAngle = kalman_get_angle(kalman, pitch, gyroYRate, dt); // Calculate the angle using a Kalman filter   

			if (abs(kalYAngle) > 90)
				gyroXRate = -gyroXRate; // Invert rate, so it fits the restriced accelerometer reading
			kalXAngle = kalman_get_angle(kalmanX, roll, gyroXRate, dt); // Calculate the angle using a Kalman filter 
			//kalXAngle = kalman_get_angle(kalman, roll, gyroXRate, dt); // Calculate the angle using a Kalman filter 
		#endif

		A += gyroXRate * dt; // Calculate gyro angle without any filter
		gyroYAngle += gyroYRate * dt;
		//A += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
		//A += kalmanY.getRate() * dt;

		compXAngle = 0.93 * (compXAngle + gyroXRate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
		compYAngle = 0.93 * (compYAngle + gyroYRate * dt) + 0.07 * pitch;

		// Reset the gyro angle when it has drifted too much
		if (A < -180 || A > 180)
			A = kalXAngle;
		if (A < -180 || A > 180)
			A = kalYAngle;

		/* Print Data */
		#if 0 // Set to 1 to activate
		//Serial.print(accX); Serial.print("\t");
		printf("%f",accY); printf("\t");
		printf("%f",accZ); printf("\t");

		printf("%f",gyroX); printf("\t");
		printf("%f",gyroY); printf("\t");
		printf("%f",gyroZ); printf("\t");

		printf("\t");
		#endif

		printf("%f",roll); printf("\t");
		printf("%f",A); printf("\t");
		printf("%f",compXAngle); printf("\t");
		printf("%f",kalXAngle); printf("\t");

		printf("\t");
		printf("%f",pitch); printf("\t");
		printf("%f",A); printf("\t");
		printf("%f",compYAngle); printf("\t");
		printf("%f",kalYAngle); printf("\t");

		#if 0 // Set to 1 to print the temperature
		Serial.print("\t");

		//double temperature = (double)tempRaw / 340.0 + 36.53;
		//Serial.print(temperature); Serial.print("\t");
		#endif

		printf("\r\n");
		vTaskDelay(pdMS_TO_TICKS(2));
	}
}

void user_init(void) {
	uart_set_baud(0, 115200);
	i2c_init(BUS_I2C, SCL, SDA, I2C_FREQ_100K);
	// fix i2c driver to work with MPU-9250
	gpio_enable(SCL, GPIO_OUTPUT);

	// turn off Wemos led
	gpio_enable(gpio_wemos_led, GPIO_OUTPUT);
	gpio_write(gpio_wemos_led, 1);


	// create pcf task
	//xTaskCreate(pcf_task, "PCF task", 1000, NULL, 2, NULL);
	//i2cData[0] = 7;
	//i2cData[1] = 0x00;
	//i2cData[2] = 0x00;
	//i2cData[3] = 0x00;
	//while (*i2cWrite(0x19, i2cData, 4, false)); // popravi naslove
	//while (i2cWrite(0x3B, 0x01, true)); // popravi naslove
	

	//while (i2cRead(0x75, i2cData, 1));
	/*
	if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
		printf("Error reading sensor");
		while (1);
	}
	*/
	vTaskDelay(pdMS_TO_TICKS(100)); // Wait for sensor to stabilize

	/* Set kalman and gyro starting angle */
	//while (i2cRead(0x3B, i2cData, 6));
	accX = read_bytes_mpu(MPU9250_ACCEL_X);
	accY = read_bytes_mpu(MPU9250_ACCEL_Y);
	accZ = read_bytes_mpu(MPU9250_ACCEL_Z);

	kalman_init(kalmanX);
	kalman_init(kalmanY);

		
	#ifdef RESTRICT_PITCH // Eq. 25 and 26
  		double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  		double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
	#else // Eq. 28 and 29
  		double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  		double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
	#endif
		
	//setAngle(kalmanX, roll); 
	setAngle(kalmanX, roll); // Set starting angle 
	//setAngle(kalmanY, pitch); 
	setAngle(kalmanY, pitch); 
	A = roll;
	A = pitch;
	compXAngle = roll;
	compYAngle = pitch;

	gettimeofday(&tval_before, NULL);

	// create mpu task
	//xTaskCreate(mpu_task, "MPU-9250 task", 1000, NULL, 2, NULL);
	xTaskCreate(loop_task, "MPU-9250 task", 1000, NULL, 2, NULL);
}