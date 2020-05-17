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

#define SMPLRT_DIV      	0x19
#define MPU_CONFIG      	0x1A
#define GYRO_CONFIG     	0x1B
#define ACCEL_CONFIG    	0x1C
#define ACCEL_CONFIG2   	0x1D
#define INT_PIN_CFG     	0x37
#define INT_ENABLE      	0x38
#define DATA_READY_MASK 	0x01
#define MAGIC_OVERFLOW_MASK 0x8
#define PWR_MGMT_2          0x6C
#define PWR_MGMT_1          0x6B

#define PWR_DOWN            0x00
#define CLOCK_SEL_PLL       0x01
#define RESET               0x01
#define PWR_RESET           0x80

#define USER_CTRL_AD 		0x6A
#define CNTL1_AD 			0x0A
#define CNTL2_AD            0x0B
#define ASAX_AD 			0x10
#define FUSE_ROM_AD         0x0F  
#define CNT_MEAS2_AD        0x16

#define AK8963_HXL          0x03

#define I2C_SLV0_CTRL       0x27
#define I2C_MST_EN          0x20     
#define I2C_SLV1_ADDR       0x28
#define I2C_SLV0_ADDR       0x25
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_DO         0x63
#define I2C_SLV0_EN         0x80

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
uint8_t _buffer[22];
float magCalibration[3] = {0, 0, 0};

uint16_t read_bytes_mpu(uint8_t address, uint8_t quantity, uint8_t * table, int size) {
	uint8_t register_address = quantity;
	i2c_slave_read(BUS_I2C, address, &register_address, table, size);
	return 1;
}

uint16_t read_bytes_mag(uint8_t quantity) {

	// high and low byte of quantity
	uint8_t data_high, data_low;
	uint8_t register_address = quantity;

	i2c_slave_read(BUS_I2C, MAG_ADDRESS, &register_address, &data_high, 1);
	register_address++;
	i2c_slave_read(BUS_I2C, MAG_ADDRESS, &register_address, &data_low, 1);

	return (data_high << 8) + data_low;
}

void write_bytes_mag(uint8_t register_address, uint8_t data) {
	i2c_slave_write(BUS_I2C, MAG_ADDRESS, &register_address, &data, 1);
}

void write_bytes_mpu(uint8_t register_address, uint8_t data) {
	i2c_slave_write(BUS_I2C, MPU_ADDRESS, &register_address, &data, 1);
}

void writeAK8963Register(uint8_t subAddress, uint8_t data){
  // set slave 0 to the AK8963 and set for write
    write_bytes_mpu(I2C_SLV0_ADDR,MAG_ADDRESS);

  // set the register to the desired AK8963 sub address 
	write_bytes_mpu(I2C_SLV0_REG, subAddress);

  // store the data for write
	write_bytes_mpu(I2C_SLV0_DO, data);

  // enable I2C and send 1 byte
	write_bytes_mpu(I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1);
  }

void initAK8963(float * destination) {

    write_bytes_mpu(USER_CTRL_AD,I2C_MST_EN);
    writeAK8963Register(CNTL1_AD, PWR_DOWN);
    vTaskDelay(pdMS_TO_TICKS(100));
    writeAK8963Register(CNTL1_AD,CNT_MEAS2_AD);
    vTaskDelay(pdMS_TO_TICKS(100));

    /*
    write_bytes_mpu(PWR_MGMT_1, PWR_RESET);
    vTaskDelay(pdMS_TO_TICKS(1));
    writeAK8963Register(CNTL2_AD, RESET);
    write_bytes_mpu(PWR_MGMT_1,CLOCK_SEL_PLL);

    write_bytes_mpu(USER_CTRL_AD,I2C_MST_EN);
    vTaskDelay(pdMS_TO_TICKS(10));
    writeAK8963Register(CNTL1_AD, PWR_DOWN);
    vTaskDelay(pdMS_TO_TICKS(100));
    writeAK8963Register(CNTL1_AD,FUSE_ROM_AD);
    vTaskDelay(pdMS_TO_TICKS(100));


    writeAK8963Register(CNTL1_AD,PWR_DOWN);
    vTaskDelay(pdMS_TO_TICKS(100));

    writeAK8963Register(CNTL1_AD,CNT_MEAS2_AD);
    vTaskDelay(pdMS_TO_TICKS(100));
    write_bytes_mpu(PWR_MGMT_1,CLOCK_SEL_PLL);
    read_bytes_mpu(MPU_ADDRESS,AK8963_HXL,&_buffer[0],7);
    */
}

uint16_t read_bytes(uint8_t quantity) {
	uint8_t data_high, data_low;
    int8_t register_address = quantity;
	i2c_slave_read(BUS_I2C, MPU_ADDRESS, &register_address, &data_low, 1);
	register_address++;
	i2c_slave_read(BUS_I2C, MPU_ADDRESS, &register_address, &data_high, 1);

	return (data_high << 8) + data_low;
}

void mpu_task(void *pvParameters) {
    read_bytes_mag(0x02);
	while (1) {
		// turn off Wemos led
        writeAK8963Register(0x3b, _buffer[0]);
        if (read_bytes_mpu(MPU_ADDRESS, 0x3b, &_buffer[0], 22) > 0) {
            /*
            printf("Accel_x: %d \n", (((int16_t)_buffer[0]) << 8) | _buffer[1]);
            printf("Accel_y: %d \n", (((int16_t)_buffer[2]) << 8) | _buffer[3]);
            printf("Accel_z: %d \n", (((int16_t)_buffer[4]) << 8) | _buffer[5]);
            printf("Temp: %d \n", (((int16_t)_buffer[6]) << 8) | _buffer[7]);
            printf("Gyro_x: %d \n", (((int16_t)_buffer[8]) << 8) | _buffer[9]);
            printf("Gyro_y: %d \n", (((int16_t)_buffer[10]) << 8) | _buffer[11]);
            printf("Gyro_z: %d \n", (((int16_t)_buffer[12]) << 8) | _buffer[13]);
            */
            write_bytes_mag(0x0A,0x01); 
            printf("Mag_x: %d \n", read_bytes(0x03));
            printf("Mag_y: %d \n", read_bytes(0x05));
            printf("Mag_z: %d \n", read_bytes(0x07));
            printf("\n");

            /*
            printf("Mag_x: %d \n", (((int16_t)_buffer[15]) << 8) | _buffer[14]);
            printf("Mag_y: %d \n", (((int16_t)_buffer[17]) << 8) | _buffer[16]);
            printf("Mag_z: %d \n", (((int16_t)_buffer[19]) << 8) | _buffer[18]);
            printf("Dest_x: %d \n", (int16_t)magCalibration[0]);
            printf("Dest_y: %d \n", (int16_t)magCalibration[1]);
            printf("Dest_z: %d \n", (int16_t)magCalibration[2]);
            printf("\n");
            */
        }
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

    write_bytes_mpu(INT_ENABLE, 0x01);
    write_bytes_mpu(0x37,0x02);
    write_bytes_mag(0x0A,0x01);

    //writeAK8963Register(CNTL1_AD,PWR_DOWN);
    //vTaskDelay(pdMS_TO_TICKS(100));

    //initAK8963(magCalibration);

	// create mpu task
	xTaskCreate(mpu_task, "MPU-9250 task", 1000, NULL, 1, NULL);
}