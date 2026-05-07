/*
 * FlightController.c
 *
 *  Created on: Apr 27, 2026
 *      Author: dalya
 */
#include "FlightController.h"


extern uint8_t ibus_dma_buffer[IBUS_FRAME_LENGTH];


void flight_controller_update(Drone_t* drone){
	float roll_correction, pitch_correction, yaw_correction;
	get_commands(drone->rc_cmd);

	compute_current_state(drone->state, drone->mpu, drone->bmp, drone->hmc);

	drone_control(drone->roll_pid, drone->pitch_pid, drone->yaw_pid, drone->rc_cmd,
			drone->state, &roll_correction, &pitch_correction, &yaw_correction);

	motor_mixer(roll_correction, pitch_correction, yaw_correction, drone->rc_cmd->throttle, drone->motor_out);

	ESC_PWM_SetThrottle(drone->motor_out->htim, drone->motor_out->motor1, drone->motor_out->motor2,
			drone->motor_out->motor3, drone->motor_out->motor4);
}


