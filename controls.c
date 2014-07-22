/*
 * $Id$
 *
 * Copyright (C) 2008-2010 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "controls.h"
#include "actuators_pwm_arch.h"
//#include "paparazzi.h"
#include "i2c_arch.h"

//#include <stdint.h>
#ifndef INT32_MIN
#define INT32_MIN (-2147483647-1)
#endif

#ifndef INT32_MAX
#define INT32_MAX (2147483647)
#endif

#define actuators actuators_pwm_values
#define Actuator(_x) actuators_pwm_values[_x]
#define ActuatorsCommit() do { } while(0);

#define PWM_GAIN_SCALE 2
#define PWM_OFF 1000
#define I2C_OFF 0

struct i2c_transaction i2c_trans_cmd;

int pending_count = 0;

int32_t servo_ailevon_left_travel_down = (SERVO_AILEVON_LEFT_TRAVEL_DOWN_NUM * 256) / SERVO_AILEVON_LEFT_TRAVEL_DOWN_DEN;
int32_t servo_ailevon_left_travel_up = (SERVO_AILEVON_LEFT_TRAVEL_UP_NUM * 256) / SERVO_AILEVON_LEFT_TRAVEL_UP_DEN;
int32_t servo_ailevon_right_travel_down = (SERVO_AILEVON_RIGHT_TRAVEL_DOWN_NUM * 256) / SERVO_AILEVON_RIGHT_TRAVEL_DOWN_DEN;
int32_t servo_ailevon_right_travel_up = (SERVO_AILEVON_RIGHT_TRAVEL_UP_NUM * 256) / SERVO_AILEVON_RIGHT_TRAVEL_UP_DEN;

int32_t previous_servo_commands[6] = { 1000, 1500, 1500, 1500, 1500, 1500 };
#define MOTOR_COMMANDS_MAX_CHANGE_UP   10
#define MOTOR_COMMANDS_MAX_CHANGE_DOWN 200
int32_t du_max_down = 1; //MOTOR_COMMANDS_MAX_CHANGE_DOWN;
int32_t du_max_up = 1; //MOTOR_COMMANDS_MAX_CHANGE_UP;
int32_t ds_max_down = 1; //100;
int32_t ds_max_up = 1; //100;

#define DIGITAL_FILTER_LENGTH 8
int32_t output_history_position = 0;
int32_t output_history[6][DIGITAL_FILTER_LENGTH];

void actuators_set_i2c_new(bool_t motors_on, int32_t commands_u[]) {

	// digital filter:
	output_history_position = (output_history_position + 1) % DIGITAL_FILTER_LENGTH;

	for (int i = 0; i < 6; i++) {
		if (commands_u[i] < 0) commands_u[i] = 0;
		if (commands_u[i] > 10000) commands_u[i] = 10000;

/*		for (int h = 0; h < DIGITAL_FILTER_LENGTH-1; h++)
			output_history[i][h] = output_history[i][h+1];*/
	}

	if (motors_on) {
		for (int i = 0; i < 6; i++) {
			actuators_pwm_values[i] = 1000 + (commands_u[i]) / 10;
			if (actuators_pwm_values[i] < 1000) actuators_pwm_values[i] = 1000;
			if (actuators_pwm_values[i] > 2000) actuators_pwm_values[i] = 2000;

			// command change limiting (TODO: moving average filter):
			if ((actuators_pwm_values[i] - previous_servo_commands[i]) > du_max_up)
				actuators_pwm_values[i] = previous_servo_commands[i] + du_max_up;
			if ((actuators_pwm_values[i] - previous_servo_commands[i]) < -du_max_down)
				actuators_pwm_values[i] = previous_servo_commands[i] - du_max_down;
			previous_servo_commands[i] = actuators_pwm_values[i];

		}
	} else {
		actuators_pwm_values[0] = 1000;
		for (int i = 1; i < 6; i++)
			actuators_pwm_values[i] = 1500;
	}
	actuators_pwm_commit();
}

struct Int32Quat stabilization_att_sum_err_quat;
struct Int32Eulers stabilization_att_sum_err;

int32_t stabilization_att_fb_cmd[COMMANDS_NB];
int32_t stabilization_att_ff_cmd[COMMANDS_NB];
int32_t stabilization_cmd[COMMANDS_NB];

/**
 * rows: u0..u5 = motors, elevons
 * cols: ypr-errors, pqr-errors, int(ypr-errors), thrust_cmd
 *       p.x,p.y,p.z, d.x,d.y,d.z, i.x,i.y,i.z, t
 */
struct Int32AttitudeGainMatrixRow gain_matrix[6] = {
		{     0,     0,     0,  3072,  1536,  3072, 0, 0, 0, 1, 0},
		{     0,     0,     0, -3072,  1536, -3072, 0, 0, 0, 1, 0},
		{     0,     0,     0,  3072, -1536, -1296, 0, 0, 0, 1, 0},
		{     0,     0,     0, -3072, -1536,  1296, 0, 0, 0, 1, 0},
		{     0,     0,     0,     0,     0,  3072, 0, 0, 0, 0, 5000},
		{     0,     0,     0,     0,     0,  3072, 0, 0, 0, 0, 5000}
		//		{  6144,  6144,  6144,  3072,  1536,  3072, 0, 0, 0, 1},
		//		{ -6144,  6144, -6144, -3072,  1536, -3072, 0, 0, 0, 1},
		//		{  6144, -6144, -2592,  3072, -1536, -1296, 0, 0, 0, 1},
		//		{ -6144, -6144,  2592, -3072, -1536,  1296, 0, 0, 0, 1},
		//		{     0,     0,  6144,     0,     0,  3072, 0, 0, 0, 0},
		//		{     0,     0,  6144,     0,     0,  3072, 0, 0, 0, 0}
};

// for airplane:
// first line is motor, only thrust is controlling this output
// 1: Lwing 2: Rwing 3:tail horz 4:tail vert
struct Int32AttitudeGainMatrixRow gain_matrix_manual[6] = {
		{     0,     0,     0,     0,     0,     0, 0, 0, 0, 1, 0},
		{  3200,    00,     0,    00,    00,     0, 0, 0, 0, 0, 5000},
		{ -3200,    00,     0,    00,    00,     0, 0, 0, 0, 0, 5000},
		{     0,  3700,     0,     0,    00,     0, 0, 0, 0, 0, 5100},
		{     0,     0,  -3200,     0,     0,    00, 0, 0, 0, 0, 5000},
		{     0,     0,     0,     0,     0,    00, 0, 0, 0, 0, 5000}
};
