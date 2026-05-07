/*
 * mpu6050.h
 *
 *  Created on: Apr 15, 2026
 *      Author: dalya
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

// MPU6050 I2C Addresses
#define MPU6050_I2C_ADDR_LOW    0x68 << 1   // AD0 = 0
#define MPU6050_I2C_ADDR_HIGH   0x69 << 1   // AD0 = 1

// Register Addresses
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_WHO_AM_I        0x75

#define MPU6050_DATA_SIZE       14  // 6 bytes accel + 2 bytes temp + 6 bytes gyro

typedef enum {
	MPU6050_STATE_INIT,
	MPU6050_STATE_READY,
	MPU6050_STATE_READING_DATA,
	MPU6050_STATE_ERROR
} MPU6050_State_t;

typedef struct {
	// Scale factors (floats grouped together for alignment)
	float accel_scale;  // from configuration
	float gyro_scale;   // from configuration
	
	// Scaled sensor data (floats grouped together)
	float accel_x, accel_y, accel_z;      // in g
	float gyro_x, gyro_y, gyro_z;         // in deg/s
	float temperature;                     // in °C
	float roll, pitch, yaw;                // in degrees (calculated from accel)

	I2C_HandleTypeDef *hi2c;
	uint8_t i2c_addr;

	MPU6050_State_t state;
	// Raw sensor data (int16_t grouped together)
	int16_t accel_x_raw, accel_y_raw, accel_z_raw;
	int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
	int16_t temp_raw;
	
	// Calibration offsets
	int16_t accel_offset_x, accel_offset_y, accel_offset_z;
	int16_t gyro_offset_x, gyro_offset_y, gyro_offset_z;
	
	// Time tracking for gyro integration
	uint32_t last_update_time;  // in milliseconds
	
	// I2C buffers
	uint8_t rx_buff[14];
} MPU6050_Handle_t;

// Function declarations
void MPU6050_Init(MPU6050_Handle_t *handle, I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);
void MPU6050_WakeUp(MPU6050_Handle_t *handle);
void MPU6050_Sleep(MPU6050_Handle_t *handle);
void MPU6050_Start_Reading(MPU6050_Handle_t *handle);
void MPU6050_Process(MPU6050_Handle_t *handle);
void MPU6050_I2C_RxCpltCallback();
void MPU6050_Calibrate(MPU6050_Handle_t *handle, uint16_t num_samples);
void MPU6050_Parse_Data(MPU6050_Handle_t *handle);
#endif /* INC_MPU6050_H_ */
