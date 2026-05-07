/*
 * mpu6050.c
 *
 *  Created on: Apr 15, 2026
 *      Author: dalya
 */

#include "mpu6050.h"
#include <math.h>
#include "main.h"

MPU6050_Handle_t *g_mpu6050_handle = NULL;
extern I2C_HandleTypeDef hi2c1;

// Forward declarations
void MPU6050_Parse_Data(MPU6050_Handle_t *handle);
static void MPU6050_Configure(MPU6050_Handle_t *handle);

/* -----------------------------------------------------------------------
 * I2C Bus Recovery
 *
 * Called BEFORE any I2C transaction on every boot/reset.
 * Fixes "I2C hangs after MCU reset" — the slave was left mid-byte when
 * the MCU reset, holding SDA low. HAL_I2C_Init() sees BUSY and never
 * recovers on its own. We bit-bang up to 9 SCL pulses to clock the slave
 * out, issue a STOP, then restore the peripheral normally.
 *
 * Pins: PB6 = SCL, PB7 = SDA  (I2C1 default on STM32F4)
 * STM32F1 users: remove the ".Alternate = GPIO_AF4_I2C1" line below.
 * ----------------------------------------------------------------------- */
static void I2C_BusRecovery(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Step 1 – DeInit the peripheral so it releases the pins */
    HAL_I2C_DeInit(hi2c);
    HAL_Delay(10);

    /* Step 2 – Enable GPIO clock and drive both lines HIGH */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);

    GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;  /* open-drain — mandatory */
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_Delay(1);

    /* Step 3 – Clock SCL up to 9 times until SDA is released by the slave */
    for (uint8_t i = 0; i < 9; i++)
    {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET)
            break;  /* SDA high — slave has released the bus */

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); /* SCL low  */
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   /* SCL high */
        HAL_Delay(1);
    }

    /* Step 4 – STOP condition: SDA low → SCL high → SDA high */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); /* SDA low  */
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   /* SCL high */
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);   /* SDA high */
    HAL_Delay(1);

    /* Step 5 – Restore pins to AF open-drain and re-init peripheral */
    GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1; /* STM32F1: delete this line */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_I2C_Init(hi2c);
    HAL_Delay(10);
}

/**
 * @brief Wake up MPU6050 from sleep mode
 * Clears the sleep bit in PWR_MGMT_1 register
 */
void MPU6050_WakeUp(MPU6050_Handle_t *handle)
{
	if (handle == NULL || handle->hi2c == NULL)
		return;

	uint8_t pwr_mgmt[2] = {MPU6050_REG_PWR_MGMT_1, 0x00};  // Clear sleep bit
	HAL_I2C_Master_Transmit(handle->hi2c, handle->i2c_addr, pwr_mgmt, 2, 100);
}

/**
 * @brief Put MPU6050 into sleep mode (low power)
 * Sets the sleep bit in PWR_MGMT_1 register
 */
void MPU6050_Sleep(MPU6050_Handle_t *handle)
{
	if (handle == NULL || handle->hi2c == NULL)
		return;

	uint8_t pwr_mgmt[2] = {MPU6050_REG_PWR_MGMT_1, 0x40};  // Set sleep bit (bit 6)
	HAL_I2C_Master_Transmit(handle->hi2c, handle->i2c_addr, pwr_mgmt, 2, 100);
}

// Initialize MPU6050 with interrupt mode
void MPU6050_Init(MPU6050_Handle_t *handle, I2C_HandleTypeDef *hi2c, uint8_t i2c_addr)
{
	handle->hi2c = hi2c;
	handle->i2c_addr = i2c_addr;
	handle->state = MPU6050_STATE_INIT;

	// Set default scale factors
	handle->accel_scale = 1.0f / 16384.0f;  // ±2g range
	handle->gyro_scale = 1.0f / 131.0f;     // ±250°/s range

	// Initialize calibration offsets
	handle->accel_offset_x = 0;
	handle->accel_offset_y = 0;
	handle->accel_offset_z = 0;
	handle->gyro_offset_x = 0;
	handle->gyro_offset_y = 0;
	handle->gyro_offset_z = 0;

	// Initialize angles and time tracking for gyro integration
	handle->roll = 0.0f;
	handle->pitch = 0.0f;
	handle->yaw = 0.0f;
	handle->last_update_time = HAL_GetTick();

	g_mpu6050_handle = handle;

	// ---- THE FIX: recover bus before touching I2C ----------------------
	I2C_BusRecovery(handle->hi2c);
	// --------------------------------------------------------------------

	uint8_t data = 0x80;
	HAL_I2C_Mem_Write(handle->hi2c,  handle->i2c_addr, 0x6B, 1, &data, 1, 100);
	HAL_Delay(100);
	data = 0x00; // Wake up
	HAL_I2C_Mem_Write(handle->hi2c,  handle->i2c_addr, 0x6B, 1, &data, 1, 100);

	uint8_t check = 0;
	if (HAL_I2C_Mem_Read(handle->hi2c, handle->i2c_addr, 0x75, 1, &check, 1, 100)){
		Error_Handler();
	}
	// Configure the sensor
	MPU6050_Configure(handle);

	handle->state = MPU6050_STATE_READY;
}

// Configure MPU6050 registers
static void MPU6050_Configure(MPU6050_Handle_t *handle)
{
	// Set config register (DLPF, sampling rate)
	uint8_t config[2] = {MPU6050_REG_CONFIG, 0x00};  // No DLPF
	HAL_I2C_Master_Transmit(handle->hi2c, handle->i2c_addr, config, 2, 100);

	// Set gyro config (±250°/s)
	uint8_t gyro_config[2] = {MPU6050_REG_GYRO_CONFIG, 0x00};
	HAL_I2C_Master_Transmit(handle->hi2c, handle->i2c_addr, gyro_config, 2, 100);

	// Set accel config (±2g)
	uint8_t accel_config[2] = {MPU6050_REG_ACCEL_CONFIG, 0x00};
	HAL_I2C_Master_Transmit(handle->hi2c, handle->i2c_addr, accel_config, 2, 100);
}

// Start reading sensor data
void MPU6050_Start_Reading(MPU6050_Handle_t *handle)
{
	if (handle->state == MPU6050_STATE_READY)
	{
		handle->state = MPU6050_STATE_READING_DATA;
		HAL_I2C_Mem_Read_IT(handle->hi2c, handle->i2c_addr, MPU6050_REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, handle->rx_buff, MPU6050_DATA_SIZE);
	}
}

// Parse raw I2C data and convert to sensor values
void MPU6050_Parse_Data(MPU6050_Handle_t *handle)
{
	uint8_t *rx = handle->rx_buff;

	// Extract raw values (big-endian)
	handle->accel_x_raw = (int16_t)((rx[0] << 8) | rx[1]);
	handle->accel_y_raw = (int16_t)((rx[2] << 8) | rx[3]);
	handle->accel_z_raw = (int16_t)((rx[4] << 8) | rx[5]);
	handle->temp_raw = (int16_t)((rx[6] << 8) | rx[7]);
	handle->gyro_x_raw = (int16_t)((rx[8] << 8) | rx[9]);
	handle->gyro_y_raw = (int16_t)((rx[10] << 8) | rx[11]);
	handle->gyro_z_raw = (int16_t)((rx[12] << 8) | rx[13]);

	// Apply calibration offsets and scale factors
	handle->accel_x = (handle->accel_x_raw - handle->accel_offset_x) * handle->accel_scale;
	handle->accel_y = (handle->accel_y_raw - handle->accel_offset_y) * handle->accel_scale;
	handle->accel_z = (handle->accel_z_raw - handle->accel_offset_z) * handle->accel_scale;

	handle->gyro_x = (handle->gyro_x_raw - handle->gyro_offset_x) * handle->gyro_scale;
	handle->gyro_y = (handle->gyro_y_raw - handle->gyro_offset_y) * handle->gyro_scale;
	handle->gyro_z = (handle->gyro_z_raw - handle->gyro_offset_z) * handle->gyro_scale;

	// Temperature: 35°C = 0, +0.00294°C / LSB
	handle->temperature = (handle->temp_raw / 340.0f) + 36.53f;

	// Calculate roll, pitch, and yaw from gyro data using integration
	uint32_t current_time = HAL_GetTick();
	float dt = (current_time - handle->last_update_time) / 1000.0f;  // Convert to seconds
	
	// Prevent dt from being too large (e.g., on first call)
	if (dt > 0.1f)
		dt = 0.01f;  // Default to 10ms if dt is unreasonable
	
	// Integrate gyro rates to get angles (degrees)
	handle->roll += handle->gyro_x * dt;
	handle->pitch += handle->gyro_y * dt;
	handle->yaw += handle->gyro_z * dt;
	
	// Update the last update time
	handle->last_update_time = current_time;
}

// Calibration function - collects samples to compute offset values
void MPU6050_Calibrate(MPU6050_Handle_t *handle, uint16_t num_samples)
{
	// Accumulation variables
	int32_t accel_x_sum = 0, accel_y_sum = 0, accel_z_sum = 0;
	int32_t gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;

	// Collect samples
	for (uint16_t i = 0; i < num_samples; i++)
	{
		// Trigger a read
		HAL_I2C_Mem_Read(handle->hi2c, handle->i2c_addr, MPU6050_REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, handle->rx_buff, MPU6050_DATA_SIZE, 100);

		handle->accel_x_raw = (int16_t)((handle->rx_buff[0] << 8) | handle->rx_buff[1]);
		handle->accel_y_raw = (int16_t)((handle->rx_buff[2] << 8) | handle->rx_buff[3]);
		handle->accel_z_raw = (int16_t)((handle->rx_buff[4] << 8) | handle->rx_buff[5]);
		handle->temp_raw = (int16_t)((handle->rx_buff[6] << 8) | handle->rx_buff[7]);
		handle->gyro_x_raw = (int16_t)((handle->rx_buff[8] << 8) | handle->rx_buff[9]);
		handle->gyro_y_raw = (int16_t)((handle->rx_buff[10] << 8) | handle->rx_buff[11]);
		handle->gyro_z_raw = (int16_t)((handle->rx_buff[12] << 8) | handle->rx_buff[13]);
		// Accumulate raw values
		accel_x_sum += handle->accel_x_raw;
		accel_y_sum += handle->accel_y_raw;
		accel_z_sum += handle->accel_z_raw;
		gyro_x_sum += handle->gyro_x_raw;
		gyro_y_sum += handle->gyro_y_raw;
		gyro_z_sum += handle->gyro_z_raw;

		// Small delay between samples
	}

	// Calculate averages and store as offsets
	handle->accel_offset_x = (int16_t)(accel_x_sum / num_samples);
	handle->accel_offset_y = (int16_t)(accel_y_sum / num_samples);
	handle->accel_offset_z = (int16_t)(accel_z_sum / num_samples);
	handle->gyro_offset_x = (int16_t)(gyro_x_sum / num_samples);
	handle->gyro_offset_y = (int16_t)(gyro_y_sum / num_samples);
	handle->gyro_offset_z = (int16_t)(gyro_z_sum / num_samples);
}
