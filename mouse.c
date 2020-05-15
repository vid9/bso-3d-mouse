#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "i2c/i2c.h"

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

typedef enum {
	BMP280_TEMPERATURE, BMP280_PRESSURE
} bmp280_quantity;

typedef enum {
	MPU9250_ACCEL_X = 0x3b,
	MPU9250_ACCEL_Y = 0x3d,
	MPU9250_ACCEL_Z = 0x3f,
	MPU9250_TEMP = 0x41,
	MPU9250_GYRO_X = 0x43,
	MPU9250_GYRO_Y = 0x45,
	MPU9250_GYRO_Z = 0x47
} mpu9250_quantity;

int accelerometer_x_values[20];
int accelerometer_y_values[20];
int accelerometer_z_values[20];

int previous_acceleration_x;
int previous_acceleration_y;
int previous_acceleration_z;

struct timeval tval_before, tval_after, tval_result;

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

		printf("Read pcf_byte");

		// button 1 is pressed
		if ((pcf_byte & button1) == 0) {
			// clear buttons states and toggle led 1
			write_byte_pcf((pcf_byte ^ ~led1) | clr_btn);
			// display temperature
			//printf("Temperature: %.2f C\n", read_bmp280(BMP280_TEMPERATURE));

			// button 2 is pressed
		} else if ((pcf_byte & button2) == 0) {
			// clear buttons states and turn on led 2
			write_byte_pcf((pcf_byte & led2) | clr_btn);
			// display pressure
			//printf("Pressure: %.2f Pa\n", read_bmp280(BMP280_PRESSURE));

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
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

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
	while (1) {
		int accelometer_x = read_bytes_mpu(MPU9250_ACCEL_X);
		int accelometer_y = read_bytes_mpu(MPU9250_ACCEL_Y);
		int accelometer_z = read_bytes_mpu(MPU9250_ACCEL_Z);

		for (int i = 0; i < 20; i++) {
			if (i == 19) {
				accelerometer_x_values[i] = accelometer_x;
				accelerometer_y_values[i] = accelometer_y;
				accelerometer_z_values[i] = accelometer_z;
			} else {
				accelerometer_x_values[i] = accelerometer_x_values[i + 1];
				accelerometer_y_values[i] = accelerometer_y_values[i + 1];
				accelerometer_z_values[i] = accelerometer_z_values[i + 1];
			}
		}

		int gyroscope_x = read_bytes_mpu(MPU9250_GYRO_X);
		int gyroscope_y = read_bytes_mpu(MPU9250_GYRO_Y);
		int gyroscope_z = read_bytes_mpu(MPU9250_GYRO_Z);

		int smoothed_accelerometer_x = 0;
		int smoothed_accelerometer_y= 0;
		int smoothed_accelerometer_z = 0;

		for (int i = 0; i < 20; i++) {
			smoothed_accelerometer_x += accelerometer_x_values[i];
			smoothed_accelerometer_y += accelerometer_y_values[i];
			smoothed_accelerometer_z += accelerometer_z_values[i];
		}

		smoothed_accelerometer_x = smoothed_accelerometer_x/20;
		smoothed_accelerometer_y = smoothed_accelerometer_y/20;
		smoothed_accelerometer_z = smoothed_accelerometer_z/20;

		gettimeofday(&tval_after, NULL);
		
		timersub(&tval_after, &tval_before, &tval_result);
		
		long dt = tval_result.tv_usec; // Calculate delta time

		printf("delta time: %ld\n", dt);

		gettimeofday(&tval_before, NULL);

		int velocity_x = (smoothed_accelerometer_x + previous_acceleration_x) / 2*dt;
		int velocity_y = (smoothed_accelerometer_y + previous_acceleration_y) / 2*dt;
		int velocity_z = (smoothed_accelerometer_z + previous_acceleration_z) / 2*dt;

		// turn off Wemos led
		gpio_write(gpio_wemos_led, 1);
		printf("Accel_x: %d | raw: %d | velocity: %d \n", smoothed_accelerometer_x, accelometer_x, velocity_x);
		printf("Accel_y: %d | raw: %d | velocity: %d \n", smoothed_accelerometer_y, accelometer_y, velocity_y);
		printf("Accel_z: %d | raw: %d | velocity: %d \n", smoothed_accelerometer_z, accelometer_z, velocity_z); printf("\n");
		//printf("Gyro_x: %d \n", gyroscope_x);
		//printf("Gyro_y: %d \n", gyroscope_y);
		//printf("Gyro_z: %d \n", gyroscope_z);

		previous_acceleration_x = smoothed_accelerometer_x;
		previous_acceleration_y = smoothed_accelerometer_y;
		previous_acceleration_z = smoothed_accelerometer_z;

		// check again after 100 ms
		vTaskDelay(pdMS_TO_TICKS(100));
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

	int accelometer_x = read_bytes_mpu(MPU9250_ACCEL_X);
	int accelometer_y = read_bytes_mpu(MPU9250_ACCEL_Y);
	int accelometer_z = read_bytes_mpu(MPU9250_ACCEL_Z);

	for (int i = 0; i < 20; i++) {
		accelerometer_x_values[i] = accelometer_x;
		accelerometer_y_values[i] = accelometer_y;
		accelerometer_z_values[i] = accelometer_z;
	}

	previous_acceleration_x = accelometer_x;
	previous_acceleration_y = accelometer_y;
	previous_acceleration_z = accelometer_z;

	// create pcf task
	xTaskCreate(pcf_task, "PCF task", 1000, NULL, 2, NULL);

	// create mpu task
	xTaskCreate(mpu_task, "MPU-9250 task", 1000, NULL, 2, NULL);
}

