/*
 * FlightController.h
 *
 *  Created on: Apr 27, 2026
 *      Author: dalya
 */

#ifndef INC_RECEIVER_H_
#define INC_RECEIVER_H_

#include "utils.h"
#include "sbus_rx.h"
#include "stm32f4xx_hal.h"

#define IBUS_USER_CHANNELS	5

#define PULSE_MIN		1000.0f
#define PULSE_MAX		2000.0f
#define PULSE_MID 		1500.0f

#define ROLL_MAX_DEG 	45.0f
#define ROLL_MIN_DEG 	-45.0f
#define PITCH_MAX_DEG	45.0f
#define PITCH_MIN_DEG	-45.0f
#define YAW_MAX_DEG  	45.0f
#define YAW_MIN_DEG  	-45.0f
#define THROTTLE_MAX	1800
#define THROTTLE_MIN	1200


typedef struct {
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	uint16_t throttle;
	Drone_state arm_value;
	uint16_t ibus_data[IBUS_FRAME_LENGTH];
}RC_Command_t;

void get_commands(RC_Command_t * rc_cmd);
void map_raw_to_commands(uint16_t roll_raw, uint16_t pitch_raw, uint16_t throttle_raw, uint16_t yaw_raw, RC_Command_t * rc_cmd);
void parse_received_data(uint16_t *ibus_data);

#endif /* INC_RECEIVER_H_ */
