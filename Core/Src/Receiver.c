/*
 * FlightController.c
 *
 *  Created on: Apr 27, 2026
 *      Author: dalya
 */

#include "Receiver.h"
#include "utils.h"
int roll, pitch,yaw;
extern uint8_t ibus_dma_buffer[IBUS_FRAME_LENGTH];
void parse_received_data(uint16_t *ibus_data){
	for (uint8_t channel_index = 0 ,buffer_index = 2; channel_index < IBUS_USER_CHANNELS ; channel_index++, buffer_index+= 2){
		ibus_data[channel_index] = ibus_dma_buffer[buffer_index] | (ibus_dma_buffer[buffer_index + 1] << 8);
	}
}

void map_raw_to_commands(uint16_t roll_raw, uint16_t pitch_raw, uint16_t throttle_raw, uint16_t yaw_raw, RC_Command_t * rc_cmd){
	rc_cmd->roll = (roll_raw - PULSE_MIN) / (PULSE_MAX - PULSE_MIN) * (ROLL_MAX_DEG - ROLL_MIN_DEG) + ROLL_MIN_DEG;
	rc_cmd->pitch = (pitch_raw - PULSE_MIN) / (PULSE_MAX - PULSE_MIN) * (PITCH_MAX_DEG - PITCH_MIN_DEG) + PITCH_MIN_DEG;
	rc_cmd->throttle = (uint16_t)((float)(throttle_raw - PULSE_MIN) / (PULSE_MAX - PULSE_MIN) * (THROTTLE_MAX - THROTTLE_MIN) + THROTTLE_MIN);
	rc_cmd->yaw = (yaw_raw - PULSE_MIN) / (PULSE_MAX - PULSE_MIN) * (YAW_MAX_DEG - YAW_MIN_DEG) + YAW_MIN_DEG;
}

void get_commands(RC_Command_t * rc_cmd){
	uint16_t roll_raw = ibus_dma_buffer[2] | (ibus_dma_buffer[3] << 8);
	uint16_t pitch_raw = ibus_dma_buffer[4] | (ibus_dma_buffer[5] << 8);
	uint16_t throttle_raw = ibus_dma_buffer[6] | (ibus_dma_buffer[7] << 8);
	uint16_t yaw_raw = ibus_dma_buffer[8] | (ibus_dma_buffer[9] << 8);
	uint16_t arm_raw = ibus_dma_buffer[10] | (ibus_dma_buffer[11] << 8);
	rc_cmd->roll  = (float)(roll_raw  - 1500) / 16.0f;
	rc_cmd->pitch = (float)(pitch_raw - 1500) / 16.0f;
	rc_cmd->yaw   = (float)(yaw_raw   - 1500) / 16.0f;
	rc_cmd->throttle = throttle_raw;
	//map_raw_to_commands(roll_raw, pitch_raw, throttle_raw, yaw_raw, rc_cmd);

	rc_cmd->arm_value = arm_raw > PULSE_MID ? ARMED : DISARMED;
}



