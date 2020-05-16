#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <sys/time.h>
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

#define MPU_ADDRESS	0x68
#define MAG_ADDRESS 0x0C
#define BUS_I2C		0
#define SCL 14
#define SDA 12

#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_DO 0x63
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV0_EN 0x80
#define I2C_READ_FLAG 0x80
#define AK8963_I2C_ADDR 0x0C
#define AK8963_CNTL1 0x0A
#define AK8963_PWR_DOWN 0x00
#define AK8963_WHO_AM_I 0x00

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

#define DATA_READY_MASK 0x01
#define MAGIC_OVERFLOW_MASK 0x8

kalman kalmanX, kalmanY;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double magX, magY, magZ;

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
	MPU9250_GYRO_Z = 0x47,
	MPU9250_MAG_X = 0x03,
	MPU9250_MAG_Y = 0x05,
	MPU9250_MAG_Z = 0x07
} mpu9250_quantity;

uint32_t timer;

const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

struct timeval tval_before, tval_after, tval_result;

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

uint16_t read_bytes_mag(mpu9250_quantity quantity) {

	// high and low byte of quantity
	uint8_t data_high, data_low;
	uint8_t register_address = (uint8_t) quantity;

	i2c_slave_read(BUS_I2C, MAG_ADDRESS, &register_address, &data_high, 1);
	register_address++;
	i2c_slave_read(BUS_I2C, MAG_ADDRESS, &register_address, &data_low, 1);

	return (data_high << 8) + data_low;
}

/*
void read_AK8963( uint8_t subAddress, uint8_t count )
{
  	// set slave 0 to the AK8963 and set for read
  	i2c_slave_write(BUS_I2C, I2C_SLV0_ADDR, NULL, AK8963_I2C_ADDR | I2C_READ_FLAG, 1);
  	// set the register to the desired AK8963 sub address
  	i2c_slave_write(BUS_I2C, I2C_SLV0_REG, NULL, &subAddress, 1);
  	// enable I2C and request the bytes
  	i2c_slave_write(BUS_I2C, I2C_SLV0_CTRL, NULL, I2C_SLV0_EN | count, 1);
  	vTaskDelay(pdMS_TO_TICKS(1));
}

void write_AK8963 ( uint8_t subAddress, uint8_t dataAK8963 ) {
  	i2c_slave_write(BUS_I2C, I2C_SLV0_ADDR, NULL, AK8963_I2C_ADDR, 1);
  	i2c_slave_write(BUS_I2C, I2C_SLV0_REG, NULL, &subAddress, 1);
  	i2c_slave_write(BUS_I2C, I2C_SLV0_DO, NULL, &dataAK8963, 1);
	i2c_slave_write(BUS_I2C, I2C_SLV0_CTRL, NULL, I2C_SLV0_EN | (uint8_t)1, 1);
  	vTaskDelay(pdMS_TO_TICKS(1));
} 
*/

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
		magX = read_bytes_mag(MPU9250_MAG_X);
		magY = read_bytes_mag(MPU9250_MAG_Y);
		magZ = read_bytes_mag(MPU9250_MAG_Z);

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
				setAngle(&kalmanX, roll); 
				compXAngle = roll;
				kalXAngle = roll;
				gyroXAngle = roll;
			} else
				kalXAngle = kalman_get_angle(&kalmanX, roll, gyroXRate, dt); // Calculate the angle using a Kalman filter  
			if (abs(kalXAngle) > 90)
				gyroYRate = -gyroYRate; // Invert rate, so it fits the restriced accelerometer reading
			kalYAngle = kalman_get_angle(&kalmanY, pitch, gyroYRate, dt); 
		#else
			// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
			if ((pitch < -90 && kalYAngle > 90) || (pitch > 90 && kalYAngle < -90)) {
				setAngle(&kalmanY, pitch); 
				compYAngle = pitch;
				kalYAngle = pitch;
				gyroYAngle = pitch;
			} else
				kalYAngle = kalman_get_angle(&kalmanY, pitch, gyroYRate, dt); // Calculate the angle using a Kalman filter   

			if (abs(kalYAngle) > 90)
				gyroXRate = -gyroXRate; // Invert rate, so it fits the restriced accelerometer reading
			kalXAngle = kalman_get_angle(&kalmanX, roll, gyroXRate, dt); // Calculate the angle using a Kalman filter 
		#endif

		gyroXAngle += gyroXRate * dt; // Calculate gyro angle without any filter
		gyroYAngle += gyroYRate * dt;
		//gyroXAngle += &kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
		//gyroYAngle += &kalmanY.getRate() * dt;

		compXAngle = 0.93 * (compXAngle + gyroXRate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
		compYAngle = 0.93 * (compYAngle + gyroYRate * dt) + 0.07 * pitch;

		// Reset the gyro angle when it has drifted too much
		if (gyroXAngle < -180 || gyroXAngle > 180)
			gyroXAngle = kalXAngle;
		if (gyroYAngle < -180 || gyroYAngle > 180)
			gyroYAngle = kalYAngle;

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
		printf("Roll: %f\n",roll);
		printf("Pitch: %f\n",pitch);
		printf("Accel_z: %f ", accY);
		printf("	Accel_x: %f ", accX);
		printf("	Accel_y: %f\n",accY);
		printf("Gyro_z: %f ", gyroZ);
		printf("	Gyro_x: %f", gyroX);
		printf("	Gyro_y: %f\n", gyroY);
		printf("Mag_z: %f ", magZ);
		printf("	Mag_x: %f", magX);
		printf("	Mag_y: %f", magY);
		/*
		printf("Roll: %f\n",roll);
		printf("%f",gyroXAngle); printf("\n");
		printf("%f",compXAngle); printf("\n");
		printf("%f",kalXAngle); printf("\n");

		printf("\n");
		printf("Pitch: %f\n",pitch); printf("\n");
		printf("%f",gyroYAngle); printf("\n");
		printf("%f",compYAngle); printf("\n");
		printf("%f",kalYAngle); printf("\n");
		*/
		#if 0 // Set to 1 to print the temperature
			Serial.print("\t");

			//double temperature = (double)tempRaw / 340.0 + 36.53;
			//Serial.print(temperature); Serial.print("\t");
		#endif

		printf("\r\n");
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}


void user_init(void) {
	uart_set_baud(0, 115200);
	i2c_init(BUS_I2C, SCL, SDA, I2C_FREQ_400K);
	// fix i2c driver to work with MPU-9250
	gpio_enable(SCL, GPIO_OUTPUT);

	// turn off Wemos led
	gpio_enable(gpio_wemos_led, GPIO_OUTPUT);
	gpio_write(gpio_wemos_led, 1);

	vTaskDelay(pdMS_TO_TICKS(100)); // Wait for sensor to stabilize
	/* Set kalman and gyro starting angle */
	accX = read_bytes_mpu(MPU9250_ACCEL_X);
	accY = read_bytes_mpu(MPU9250_ACCEL_Y);
	accZ = read_bytes_mpu(MPU9250_ACCEL_Z);
	gyroX = read_bytes_mpu(MPU9250_GYRO_X);
	gyroY = read_bytes_mpu(MPU9250_GYRO_Y);
	gyroZ = read_bytes_mpu(MPU9250_GYRO_Z);
	magX = read_bytes_mag(MPU9250_MAG_X);
	magY = read_bytes_mag(MPU9250_MAG_Y);
	magZ = read_bytes_mag(MPU9250_MAG_Z);

	kalman_init(&kalmanX);
	kalman_init(&kalmanY);
	
	#ifdef RESTRICT_PITCH // Eq. 25 and 26
  		double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  		double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
	#else // Eq. 28 and 29
  		double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  		double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
	#endif
	
	setAngle(&kalmanX, roll); // Set starting angle 
	setAngle(&kalmanY, pitch);
	
	gyroXAngle = roll;
	gyroYAngle = pitch;
	compXAngle = roll;
	compYAngle = pitch;

//	write_AK8963(AK8963_CNTL1, AK8963_PWR_DOWN);
//	vTaskDelay(pdMS_TO_TICKS(100));

//	read_AK8963(AK8963_WHO_AM_I, 1);
//	vTaskDelay(pdMS_TO_TICKS(500)); // giving the AK8963 lots of time to recover from reset

	gettimeofday(&tval_before, NULL);

	// create loop task
	xTaskCreate(loop_task, "MPU-9250 loop", 1000, NULL, 2, NULL);
}