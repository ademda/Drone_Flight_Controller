/*
 * esc_pwm.c
 *
 *  Created on: Apr 16, 2026
 *      Author: dalya
 */

#include "esc_pwm.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "Motor.h"
extern Motor_t motors;
// Utility macros for min/max
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

/**
 * Initialize ESC PWM outputs on Timer1
 * Configure all 4 channels for PWM output
 */
bool ESC_PWM_Init(TIM_HandleTypeDef *htim)
{
	if (htim == NULL)
		return false;
	
	motors.htim = htim;
	// Start PWM generation on all 4 channels
	HAL_TIM_PWM_Start(htim, ESC_TIMER_CH1);
	HAL_TIM_PWM_Start(htim, ESC_TIMER_CH2);
	HAL_TIM_PWM_Start(htim, ESC_TIMER_CH3);
	HAL_TIM_PWM_Start(htim, ESC_TIMER_CH4);

	ESC_PWM_SetThrottle(htim, ESC_PWM_MIN,ESC_PWM_MIN, ESC_PWM_MIN,ESC_PWM_MIN);
	return true;
}

/**
 * Set PWM pulse width (direct pulse value)
 */
void ESC_PWM_SetPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse)
{
	if (htim == NULL)
		return;
	__HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

/**
 * Set throttle for all 4 motors (0.0 to 1.0)
 */
void ESC_PWM_SetThrottle(TIM_HandleTypeDef *htim, uint16_t throttle1, uint16_t throttle2, uint16_t throttle3, uint16_t throttle4)
{
	if (htim == NULL)
		return;

	// Set PWM for each motor+
	ESC_PWM_SetPulse(htim, ESC_TIMER_CH1, US_TO_PULSE(throttle1));
	ESC_PWM_SetPulse(htim, ESC_TIMER_CH2, US_TO_PULSE(throttle2));
	ESC_PWM_SetPulse(htim, ESC_TIMER_CH3, US_TO_PULSE(throttle3));
	ESC_PWM_SetPulse(htim, ESC_TIMER_CH4, US_TO_PULSE(throttle4));
}

/**
 * Calibrate ESCs - ESCs must be calibrated before use
 * Procedure: Send max throttle (2000us), then min throttle (1000us)
 */
void ESC_PWM_Calibrate(TIM_HandleTypeDef *htim, uint32_t duration_ms)
{
	if (htim == NULL)
		return;

	// Step 1: Send maximum throttle (2000us) for calibration
	ESC_PWM_SetThrottle(htim, 1000, 1000, 1000, 1000);
	HAL_Delay(duration_ms);

	// Step 2: Send minimum throttle (1000us) for calibration
	ESC_PWM_SetThrottle(htim, 2000, 2000, 2000, 2000);
	HAL_Delay(duration_ms);

	// After calibration, ESCs should be ready to receive commands
	// Set to neutral (1500us) before starting
	ESC_PWM_SetThrottle(htim, 2000, 2000, 2000, 2000);
	HAL_Delay(500);
}
