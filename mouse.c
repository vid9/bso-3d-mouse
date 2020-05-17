#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "i2c/i2c.h"
#include "math.h"

#define PCF_ADDRESS	0x38
#define MPU_ADDRESS	0x68
#define MAG_ADDRESS 0x0C
#define BUS_I2C		0
#define SCL 14
#define SDA 12

#define SMPLRT_DIV      0x19
#define MPU_CONFIG      0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define ACCEL_CONFIG2   0x1D
#define INT_PIN_CFG     0x37
#define INT_ENABLE      0x38
#define PWR_MGMT      	0x6C
#define PWR_MGMT_1  	0x6B

#define AK8963_ADDRESS  0x0C
#define AK8963_RA_HXL   0x03
#define AK8963_RA_CNTL1 0x0A
#define AK8963_RA_ASAX  0x10

#define MAG_MODE_POWERDOWN        0x0
#define MAG_MODE_SINGLE           0x1
#define MAG_MODE_CONTINUOUS_8HZ   0x2
#define MAG_MODE_EXTERNAL         0x4
#define MAG_MODE_CONTINUOUS_100HZ 0x6
#define MAG_MODE_SELFTEST         0x8
#define MAG_MODE_FUSEROM          0xF

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

uint8_t magBuf[7];
uint8_t magXAdjust, magYAdjust, magZAdjust;
int16_t magXOffset, magYOffset, magZOffset;

const float Pi = 3.14159;

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

#define accelerometer_scale 2000.0

#define accelerometer_x_bias 0.0
#define accelerometer_y_bias 0.0
#define accelerometer_z_bias 500.0

#define gyroscope_x_bias 0.0
#define gyroscope_y_bias 0.0
#define gyroscope_z_bias 0.0

#define gyroscope_rotation_threshold 0.005
#define gyroscope_scale 16384.0

#define accelerometer_cache_size 20
#define gyroscope_cache_size 5
#define magnetometer_cache_size 2


double accelerometer_x_values[accelerometer_cache_size];
double accelerometer_y_values[accelerometer_cache_size];
double accelerometer_z_values[accelerometer_cache_size];

double previous_accelerometer_x;
double previous_accelerometer_y;
double previous_accelerometer_z;

double gyroscope_x_values[gyroscope_cache_size];
double gyroscope_y_values[gyroscope_cache_size];
double gyroscope_z_values[gyroscope_cache_size];

double previous_gyroscope_x;
double previous_gyroscope_y;
double previous_gyroscope_z;

double pitch_gyroscope = 0;
double roll_gyroscope = 0;
double yaw_gyroscope = 0;

double magnetometer_x_values[magnetometer_cache_size];
double magnetometer_y_values[magnetometer_cache_size];
double magnetometer_z_values[magnetometer_cache_size];

double previous_magnetometer_x;
double previous_magnetometer_y;
double previous_magnetometer_z;

bool button_1_pressed = 0;
bool button_2_pressed = 0;
bool button_3_pressed = 0;

int time_since_boot = 0;

// read 2 bytes from MAGNETOMETER on I2C bus
uint16_t read_bytes_mag(mpu9250_quantity quantity) {

	// high and low byte of quantity
	uint8_t data_high, data_low;
	uint8_t register_address = (uint8_t) quantity;

	i2c_slave_read(BUS_I2C, MAG_ADDRESS, &register_address, &data_low, 1);
	register_address++;
	i2c_slave_read(BUS_I2C, MAG_ADDRESS, &register_address, &data_high, 1);

	return (data_high << 8) + data_low;
}

void read_bytes_mag_n(uint8_t register_address, uint8_t * data, int size) {
	i2c_slave_read(BUS_I2C, MAG_ADDRESS, &register_address, data, size);
}

void write_bytes_mag(uint8_t register_address, uint8_t data) {
	i2c_slave_write(BUS_I2C, MAG_ADDRESS, &register_address, &data, 1);
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

void write_bytes_mpu(uint8_t register_address, uint8_t data) {
	i2c_slave_write(BUS_I2C, MPU_ADDRESS, &register_address, &data, 1);
}


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

	while (1) {
		pcf_byte = read_byte_pcf();

		button_1_pressed = (pcf_byte & button1) == 0;
		button_2_pressed = (pcf_byte & button2) == 0;
		button_3_pressed = (pcf_byte & button3) == 0;

		uint8_t lled = leds_off;

		if (button_1_pressed == 1) {
			lled &= led1; 
		} else {
			lled |= 0x08;
		}

		if (button_2_pressed == 1) {
			lled &= led2; 
		} else {
			lled |= 0x02;
		}

		if (button_3_pressed == 1) {
			lled &= led3;
		} else {
			lled |= 0x04;
		}
		
		
		write_byte_pcf(lled);

		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

int16_t mag_get(uint8_t highIndex, uint8_t lowIndex) {
  return (((int16_t) magBuf[highIndex]) << 8) | magBuf[lowIndex];
}

float adjust_mag_value(int16_t value, uint8_t adjust) {
  return ((float) value * (((((float) adjust - 128) * 0.5) / 128) + 1));
}

double magZ() {
  return adjust_mag_value(mag_get(5, 4), magZAdjust) + magZOffset;
}

double magY() {
  return adjust_mag_value(mag_get(3, 2), magYAdjust) + magYOffset;
}

double magX() {
  return adjust_mag_value(mag_get(1, 0), magXAdjust) + magXOffset;
}

void mag_set_mode(uint8_t mode) {
  write_bytes_mag(AK8963_RA_CNTL1, mode);
  vTaskDelay(pdMS_TO_TICKS(10));
}

void mag_read_adjust_values() {
  mag_set_mode(MAG_MODE_POWERDOWN);
  mag_set_mode(MAG_MODE_FUSEROM);
  uint8_t buff[3];
  read_bytes_mag_n(AK8963_RA_ASAX, &buff[0], 3);
  magXAdjust = buff[0];
  magYAdjust = buff[1];
  magZAdjust = buff[2];
}

void mag_wakeup() {
  unsigned char bits;
  bits = read_bytes_mpu(PWR_MGMT_1);
  bits &= 0x70; // Turn off SLEEP, STANDBY, CYCLE
  write_bytes_mpu(PWR_MGMT_1, bits);
  vTaskDelay(pdMS_TO_TICKS(10));
}

void mag_enable_slave_mode() {
  unsigned char bits;
  bits = read_bytes_mpu(INT_PIN_CFG);
  bits |= 0x1F; // Activate BYPASS_EN
  write_bytes_mpu(INT_PIN_CFG, bits);
  vTaskDelay(pdMS_TO_TICKS(10));
}


float mag_horiz_direction() {
  return atan2(magX(), magY()) * 180 / Pi;
}

void mag_update() {
  read_bytes_mag_n(AK8963_RA_HXL, &magBuf[0], 7);
}

void begin_mag(uint8_t mode) {
  //beginWireIfNull();
  mag_wakeup();
  mag_enable_slave_mode();

  mag_read_adjust_values();
  mag_set_mode(MAG_MODE_POWERDOWN);
  mag_set_mode(mode);
  vTaskDelay(pdMS_TO_TICKS(10));
}



// check MPU-9250 sensor values
void mpu_task(void *pvParameters) {
	while (1) {
		
		int time = xTaskGetTickCount() * portTICK_PERIOD_MS;

		double accelerometer_x = ((double) read_bytes_mpu(MPU9250_ACCEL_X) - accelerometer_x_bias) / accelerometer_scale;
		double accelerometer_y = ((double) read_bytes_mpu(MPU9250_ACCEL_Y) - accelerometer_y_bias) / accelerometer_scale;
		double accelerometer_z = ((double) read_bytes_mpu(MPU9250_ACCEL_Z) - accelerometer_z_bias) / accelerometer_scale;

		if (accelerometer_x > 16) {
			accelerometer_x -= 32.76;
		}

		if (accelerometer_y > 16) {
			accelerometer_y -= 32.76;
		}

		if (accelerometer_z > 16) {
			accelerometer_z -= 32.76;
		}

		for (int i = 0; i < accelerometer_cache_size; i++) {
			if (i == (accelerometer_cache_size - 1)) {
				accelerometer_x_values[i] = accelerometer_x;
				accelerometer_y_values[i] = accelerometer_y;
				accelerometer_z_values[i] = accelerometer_z;
			} else {
				accelerometer_x_values[i] = accelerometer_x_values[i + 1];
				accelerometer_y_values[i] = accelerometer_y_values[i + 1];
				accelerometer_z_values[i] = accelerometer_z_values[i + 1];
			}
		}

		double smoothed_accelerometer_x = 0;
		double smoothed_accelerometer_y= 0;
		double smoothed_accelerometer_z = 0;

		for (int i = 0; i < accelerometer_cache_size; i++) {
			smoothed_accelerometer_x += accelerometer_x_values[i];
			smoothed_accelerometer_y += accelerometer_y_values[i];
			smoothed_accelerometer_z += accelerometer_z_values[i];
		}

		smoothed_accelerometer_x = smoothed_accelerometer_x/accelerometer_cache_size;
		smoothed_accelerometer_y = smoothed_accelerometer_y/accelerometer_cache_size;
		smoothed_accelerometer_z = smoothed_accelerometer_z/accelerometer_cache_size;

		//printf("Accel_x: %f | raw: %f | delta from previous: %f\n", smoothed_accelerometer_x, accelerometer_x, smoothed_accelerometer_x - previous_accelerometer_x);
		//printf("Accel_y: %f | raw: %f | delta from previous: %f\n", smoothed_accelerometer_y, accelerometer_y, smoothed_accelerometer_y - previous_accelerometer_y);
		//printf("Accel_z: %f | raw: %f | delta from previous: %f\n", smoothed_accelerometer_z, accelerometer_z, smoothed_accelerometer_z - previous_accelerometer_z); printf("\n");

		double roll_accelerometer = 0.0;

		// roll surges when the pitch is at 90 percent, this is a hack to fix that issue
		if (fabs(smoothed_accelerometer_y) < 0.1 && fabs(smoothed_accelerometer_z) < 0.1) {
			roll_accelerometer = 0.0;
		} else {
			roll_accelerometer = atan2(smoothed_accelerometer_y, smoothed_accelerometer_z) * 180/M_PI;
		}

		double pitch_accelerometer = atan2(-smoothed_accelerometer_x, sqrt(smoothed_accelerometer_y*smoothed_accelerometer_y + smoothed_accelerometer_z*smoothed_accelerometer_z)) * 180/M_PI;

		previous_accelerometer_x = smoothed_accelerometer_x;
		previous_accelerometer_y = smoothed_accelerometer_y;
		previous_accelerometer_z = smoothed_accelerometer_z;

		double gyroscope_x = (double) read_bytes_mpu(MPU9250_GYRO_X);
		double gyroscope_y = (double) read_bytes_mpu(MPU9250_GYRO_Y);
		double gyroscope_z = (double) read_bytes_mpu(MPU9250_GYRO_Z);

		if (gyroscope_x > 32768) {
			gyroscope_x -= 65536;
		}

		if (gyroscope_y > 32768) {
			gyroscope_y -= 65536;
		}

		if (gyroscope_z > 32768) {
			gyroscope_z -= 65536;
		}

		for (int i = 0; i < gyroscope_cache_size; i++) {
			if (i == (gyroscope_cache_size - 1)) {
				gyroscope_x_values[i] = gyroscope_x;
				gyroscope_y_values[i] = gyroscope_y;
				gyroscope_z_values[i] = gyroscope_z;
			} else {
				gyroscope_x_values[i] = gyroscope_x_values[i + 1];
				gyroscope_y_values[i] = gyroscope_y_values[i + 1];
				gyroscope_z_values[i] = gyroscope_z_values[i + 1];
			}
		}

		double smoothed_gyroscope_x = 0;
		double smoothed_gyroscope_y = 0;
		double smoothed_gyroscope_z = 0;

		for (int i = 0; i < gyroscope_cache_size; i++) {
			smoothed_gyroscope_x += gyroscope_x_values[i];
			smoothed_gyroscope_y += gyroscope_y_values[i];
			smoothed_gyroscope_z += gyroscope_z_values[i];
		}

		smoothed_gyroscope_x = smoothed_gyroscope_x/gyroscope_cache_size;
		smoothed_gyroscope_y = smoothed_gyroscope_y/gyroscope_cache_size;
		smoothed_gyroscope_z = smoothed_gyroscope_z/gyroscope_cache_size;

		//printf("Gyro_x: %d | raw: %d | delta from previous: %d\n", smoothed_gyroscope_x, gyroscope_x, smoothed_gyroscope_x - previous_gyroscope_x);
		//printf("Gyro_y: %d | raw: %d | delta from previous: %d\n", smoothed_gyroscope_y, gyroscope_y, smoothed_gyroscope_y - previous_gyroscope_y);
		//printf("Gyro_z: %d | raw: %d | delta from previous: %d\n", smoothed_gyroscope_z, gyroscope_z, smoothed_gyroscope_z - previous_gyroscope_z);
		
		printf("\n");
		int dt = time - time_since_boot;

		//printf("delta: %f\n", ((smoothed_gyroscope_x*dt)/gyroscope_scale) - (previous_gyroscope_x*dt)/gyroscope_scale);
		//printf("delta: %f\n", ((smoothed_gyroscope_y*dt)/gyroscope_scale) - (previous_gyroscope_y*dt)/gyroscope_scale);
		//printf("delta: %f\n", ((smoothed_gyroscope_z*dt)/gyroscope_scale) - (previous_gyroscope_z*dt)/gyroscope_scale);

		double gyroscope_x_change = (smoothed_gyroscope_x*dt)/gyroscope_scale;
		double gyroscope_y_change = (smoothed_gyroscope_y*dt)/gyroscope_scale;
		double gyroscope_z_change = (smoothed_gyroscope_z*dt)/gyroscope_scale;

		if (fabs(gyroscope_x_change) > gyroscope_rotation_threshold) {
			roll_gyroscope += gyroscope_x_change;
		}

		if (fabs(gyroscope_y_change) > gyroscope_rotation_threshold) {
			pitch_gyroscope += gyroscope_y_change;
		}

		if (fabs(gyroscope_z_change) > gyroscope_rotation_threshold) {
			yaw_gyroscope -= gyroscope_z_change;
		}

		previous_gyroscope_x = smoothed_gyroscope_x;
		previous_gyroscope_y = smoothed_gyroscope_y;
		previous_gyroscope_z = smoothed_gyroscope_z;

		mag_update();
		double magnetometer_x = magX();
		double magnetometer_y = magY();
		double magnetometer_z = magZ();

		for (int i = 0; i < magnetometer_cache_size; i++) {
			if (i == (magnetometer_cache_size - 1)) {
				magnetometer_x_values[i] = magnetometer_x;
				magnetometer_y_values[i] = magnetometer_y;
				magnetometer_z_values[i] = magnetometer_z;
			} else {
				magnetometer_x_values[i] = magnetometer_x_values[i + 1];
				magnetometer_y_values[i] = magnetometer_y_values[i + 1];
				magnetometer_z_values[i] = magnetometer_z_values[i + 1];
			}
		}

		double smoothed_magnetometer_x = 0;
		double smoothed_magnetometer_y = 0;
		double smoothed_magnetometer_z = 0;

		for (int i = 0; i < magnetometer_cache_size; i++) {
			smoothed_magnetometer_x += magnetometer_x_values[i];
			smoothed_magnetometer_y += magnetometer_y_values[i];
			smoothed_magnetometer_z += magnetometer_z_values[i];
		}

		smoothed_magnetometer_x = smoothed_magnetometer_x/magnetometer_cache_size;
		smoothed_magnetometer_y = smoothed_magnetometer_y/magnetometer_cache_size;
		smoothed_magnetometer_z = smoothed_magnetometer_z/magnetometer_cache_size;

		printf("Mag_x: %f | raw: %f | delta from previous: %f\n", smoothed_magnetometer_x, magnetometer_x, smoothed_magnetometer_x - previous_magnetometer_x);
		printf("Mag_y: %f | raw: %f | delta from previous: %f\n", smoothed_magnetometer_y, magnetometer_y, smoothed_magnetometer_y - previous_magnetometer_y);
		printf("Mag_z: %f | raw: %f | delta from previous: %f\n", smoothed_magnetometer_z, magnetometer_z, smoothed_magnetometer_z - previous_magnetometer_z);
		
		printf("\n");
		
		previous_magnetometer_x = smoothed_magnetometer_x;
		previous_magnetometer_y = smoothed_magnetometer_y;
		previous_magnetometer_z = smoothed_magnetometer_z;

		vTaskDelay(pdMS_TO_TICKS(100));
		//printf("ROLL - acc: %f, gyr: %f\n", roll_accelerometer, roll_gyroscope);
		//printf("PITCH - acc: %f, gyr: %f\n", pitch_accelerometer, pitch_gyroscope);
		//printf("YAW - gyr: %f\n", yaw_gyroscope);

		//printf("%f:%f:%f", 0.5*roll_accelerometer + 0.5*roll_gyroscope, 0.5*pitch_accelerometer + 0.5*pitch_gyroscope, yaw_gyroscope);
		printf("\n");
	
		//printf("\n");

		time_since_boot = time;

		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

void init_mpu() {

	write_bytes_mpu(PWR_MGMT, 0);

    // bit0 1: Set LPF to 184Hz 0: No LPF, bit6 Stop if fifo full
    write_bytes_mpu(MPU_CONFIG, (1<<6) | 1);

    // Sample rate 1000Hz
    write_bytes_mpu(SMPLRT_DIV, 0);

    // Gyro 2000dps
    write_bytes_mpu(GYRO_CONFIG, 3<<3);

    // Accel full scale 16g
    write_bytes_mpu(ACCEL_CONFIG, 3<<3);

    // Set LPF to 218Hz BW
    write_bytes_mpu(ACCEL_CONFIG2, 1);

    //write_bytes_mag(USER_CTRL_AD, 1);
	//write_bytes_mag(0X00, 1);

	//write_bytes_mag(INT_PIN_CFG, 1); //0000 0010 in binary, turn on the bypass multiplexer
	//write_bytes_mag(0x02,true);

	write_bytes_mag(CNTL1_AD, 0x00);
	vTaskDelay(pdMS_TO_TICKS(100));
	write_bytes_mag(CNTL1_AD, 0x0F);
	vTaskDelay(pdMS_TO_TICKS(100));
	//write_bytes_mag(CNTL1_AD, 0x16);
	write_bytes_mag(CNTL1_AD,0x1F);

	vTaskDelay(pdMS_TO_TICKS(100));

	write_bytes_mag(MAG_ADDRESS, 1);
	write_bytes_mag(MPU9250_MAG_X, 1);

    uint8_t val;
    // INT enable on RDY

    write_bytes_mpu(INT_ENABLE, 1);

    val = read_bytes_mpu(INT_PIN_CFG);
    val |= 0x30;
	
    write_bytes_mpu(INT_PIN_CFG, val);
}

void init_mag() {
	begin_mag(MAG_MODE_CONTINUOUS_8HZ);
    magXOffset = -20;
    magYOffset = 85;
    magZOffset = -70;
}

void user_init(void) {

	uart_set_baud(0, 115200);
	i2c_init(BUS_I2C, SCL, SDA, I2C_FREQ_400K);
	// fix i2c driver to work with MPU-9250
	gpio_enable(SCL, GPIO_OUTPUT);

	// turn off Wemos led
	gpio_enable(gpio_wemos_led, GPIO_OUTPUT);
	gpio_write(gpio_wemos_led, 1);

	init_mpu();
	init_mag();

	double accelerometer_x = ((double) read_bytes_mpu(MPU9250_ACCEL_X) - accelerometer_x_bias) / accelerometer_scale;
	double accelerometer_y = ((double) read_bytes_mpu(MPU9250_ACCEL_Y) - accelerometer_y_bias) / accelerometer_scale;
	double accelerometer_z = ((double) read_bytes_mpu(MPU9250_ACCEL_Z) - accelerometer_z_bias) / accelerometer_scale;

	if (accelerometer_x > 16) {
		accelerometer_x -= 32.76;
	}

	if (accelerometer_y > 16) {
		accelerometer_y -= 32.76;
	}

	if (accelerometer_z > 16) {
		accelerometer_z -= 32.76;
	}

	for (int i = 0; i < accelerometer_cache_size; i++) {
		accelerometer_x_values[i] = accelerometer_x;
		accelerometer_y_values[i] = accelerometer_y;
		accelerometer_z_values[i] = accelerometer_z;
	}

	previous_accelerometer_x = accelerometer_x;
	previous_accelerometer_y = accelerometer_y;
	previous_accelerometer_z = accelerometer_z;

	double roll_accelerometer = 0.0;

	// roll surges when the pitch is at 90 percent, this is a hack to fix that issue
	if (fabs(accelerometer_y) < 0.1 && fabs(accelerometer_z) < 0.1) {
		roll_accelerometer = 0.0;
	} else {
		roll_accelerometer = atan2(accelerometer_x, accelerometer_z) * 180/M_PI;
	}

	double pitch_accelerometer = atan2(-accelerometer_x, sqrt(accelerometer_y*accelerometer_y + accelerometer_z*accelerometer_z)) * 180/M_PI;

	// Fixes issue when not starting in a horizontal plane
	pitch_gyroscope = pitch_accelerometer;
	roll_gyroscope = roll_accelerometer;

	double gyroscope_x = (double) read_bytes_mpu(MPU9250_GYRO_X);
	double gyroscope_y = (double) read_bytes_mpu(MPU9250_GYRO_Y);
	double gyroscope_z = (double) read_bytes_mpu(MPU9250_GYRO_Z);

	if (gyroscope_x > 32768) {
		gyroscope_x -= 65536;
	}

	if (gyroscope_y > 32768) {
		gyroscope_y -= 65536;
	}

	if (gyroscope_z > 32768) {
		gyroscope_z -= 65536;
	}

	for (int i = 0; i < gyroscope_cache_size; i++) {
		gyroscope_x_values[i] = gyroscope_x;
		gyroscope_y_values[i] = gyroscope_y;
		gyroscope_z_values[i] = gyroscope_z;
	}

	previous_gyroscope_x = gyroscope_x;
	previous_gyroscope_y = gyroscope_y;
	previous_gyroscope_z = gyroscope_z;

	//double magnetometer_x = read_bytes_mag(MPU9250_MAG_X);
	//double magnetometer_y = read_bytes_mag(MPU9250_MAG_Y);
	//double magnetometer_z = read_bytes_mag(MPU9250_MAG_Z);

	double magnetometer_x = magX();
	double magnetometer_y = magY();
	double magnetometer_z = magZ();

	for (int i = 0; i < magnetometer_cache_size; i++) {
		if (i == (magnetometer_cache_size - 1)) {
			magnetometer_x_values[i] = magnetometer_x;
			magnetometer_y_values[i] = magnetometer_y;
			magnetometer_z_values[i] = magnetometer_z;
		} else {
			magnetometer_x_values[i] = magnetometer_x_values[i + 1];
			magnetometer_y_values[i] = magnetometer_y_values[i + 1];
			magnetometer_z_values[i] = magnetometer_z_values[i + 1];
		}
	}

	previous_magnetometer_x = magnetometer_x;
	previous_magnetometer_y = magnetometer_y;
	previous_magnetometer_z = magnetometer_z;

	// create pcf task
	xTaskCreate(pcf_task, "PCF task", 1000, NULL, 2, NULL);

	// create mpu task
	xTaskCreate(mpu_task, "MPU-9250 task", 1000, NULL, 1, NULL);
}