/*
 * esc_pwm.h
 *
 *  Created on: Apr 16, 2026
 *      Author: dalya
 */

#ifndef INC_ESC_PWM_H_
#define INC_ESC_PWM_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define US_TO_PULSE(us) ((us) * 3273/1000) // Assuming 20ms period for 50Hz PWM

// PWM pulse widths for ESC (in microseconds)
#define ESC_PWM_MIN_US          1000    // Minimum throttle
#define ESC_PWM_MAX_US          2000    // Maximum throttle
#define ESC_PWM_MID_US          1500    // Middle throttle

// Convert microseconds to timer pulse value

#define ESC_PWM_MIN             1000
#define ESC_PWM_MAX             1000
#define ESC_PWM_MID             1000

// Timer and channels for 4 ESCs
#define ESC_TIMER               TIM1
#define ESC_TIMER_CH1           TIM_CHANNEL_1    // Motor 1
#define ESC_TIMER_CH2           TIM_CHANNEL_2    // Motor 2
#define ESC_TIMER_CH3           TIM_CHANNEL_3    // Motor 3
#define ESC_TIMER_CH4           TIM_CHANNEL_4    // Motor 4

bool ESC_PWM_Init(TIM_HandleTypeDef *htim);
void ESC_PWM_SetPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse);
void ESC_PWM_SetThrottle(TIM_HandleTypeDef *htim, uint16_t throttle1, uint16_t throttle2, uint16_t throttle3, uint16_t throttle4);
void ESC_PWM_Calibrate(TIM_HandleTypeDef *htim, uint32_t duration_ms);

#endif /* INC_ESC_PWM_H_ */
