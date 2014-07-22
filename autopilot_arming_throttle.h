/*
 * Copyright (C) 2012 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file firmwares/rotorcraft/autopilot_arming_throttle.h
 *
 * Automatically arm the motors when applying throttle.
 *
 */

#ifndef AUTOPILOT_ARMING_THROTTLE_H
#define AUTOPILOT_ARMING_THROTTLE_H

//#include "autopilot_rc_helpers.h"
#include "airframe.h"
#include "radio_control.h"

#define AUTOPILOT_THROTTLE_THRESHOLD      (MAX_PPRZ / 20)
#define AUTOPILOT_YAW_THRESHOLD           (MAX_PPRZ * 19 / 20)
#ifndef AUTOPILOT_STICK_CENTER_THRESHOLD
#define AUTOPILOT_STICK_CENTER_THRESHOLD  (MAX_PPRZ * 1 / 20)
#endif

#define THROTTLE_STICK_DOWN()                                           \
  (radio_control.values[RADIO_THROTTLE] < AUTOPILOT_THROTTLE_THRESHOLD)
#define YAW_STICK_PUSHED()                                      \
  (radio_control.values[RADIO_YAW] > AUTOPILOT_YAW_THRESHOLD ||  \
   radio_control.values[RADIO_YAW] < -AUTOPILOT_YAW_THRESHOLD)
#define YAW_STICK_CENTERED()                                            \
  (radio_control.values[RADIO_YAW] < AUTOPILOT_STICK_CENTER_THRESHOLD && \
   radio_control.values[RADIO_YAW] > -AUTOPILOT_STICK_CENTER_THRESHOLD)
#define PITCH_STICK_CENTERED()                                          \
  (radio_control.values[RADIO_PITCH] < AUTOPILOT_STICK_CENTER_THRESHOLD && \
   radio_control.values[RADIO_PITCH] > -AUTOPILOT_STICK_CENTER_THRESHOLD)
#define ROLL_STICK_CENTERED()                                           \
  (radio_control.values[RADIO_ROLL] < AUTOPILOT_STICK_CENTER_THRESHOLD && \
   radio_control.values[RADIO_ROLL] > -AUTOPILOT_STICK_CENTER_THRESHOLD)

static inline bool_t rc_attitude_sticks_centered(void) {
  return ROLL_STICK_CENTERED() && PITCH_STICK_CENTERED() && YAW_STICK_CENTERED();
}

#ifdef RADIO_KILL_SWITCH
static inline bool_t kill_switch_is_on(void) {
  if (radio_control.values[RADIO_KILL_SWITCH] < 0)
    return TRUE;
  else
    return FALSE;
}
#else
static inline bool_t kill_switch_is_on(void) {
  return FALSE;
}
#endif

static inline uint8_t percent_from_rc(int channel)
{
  int per = (MAX_PPRZ + (int32_t)radio_control.values[channel]) * 50 / MAX_PPRZ;
  if (per < 0)
    per = 0;
  else if (per > 100)
    per = 100;
  return per;
}



extern bool_t autopilot_motors_on;

#define AUTOPILOT_ARMING_DELAY 10

enum arming_throttle_state {
	STATE_UNINIT,
	STATE_WAITING,
	STATE_MOTORS_OFF_READY,
	STATE_ARMING,
	STATE_MOTORS_ON,
	STATE_UNARMING
};

enum arming_throttle_state autopilot_arming_state;
uint8_t autopilot_arming_delay_counter;
bool_t autopilot_unarmed_in_auto;

static inline void autopilot_arming_init(void) {
	autopilot_arming_state = STATE_UNINIT;
	autopilot_arming_delay_counter = 0;
	autopilot_unarmed_in_auto = FALSE;
}

static inline void autopilot_arming_set(bool_t motors_on) {
	if (motors_on) {
		autopilot_arming_state = STATE_MOTORS_ON;
	} else {
		if (autopilot_arming_state == STATE_MOTORS_ON) {
			autopilot_arming_state = STATE_WAITING;
		}
	}
}

/**
 * State machine to check if motors should be turned ON or OFF.
 * - automatically unkill when applying throttle
 * - if throttle was not down at startup, you need to put throttle down again first
 * - other sticks need to be centered to start motors
 * - need to be in manual mode to start the motors
 * - AHRS needs to be aligned
 */
static inline void autopilot_arming_check_motors_on(void) {

		switch (autopilot_arming_state) {
		case STATE_UNINIT:
			//_write(1, "motors initialized ", 19);
			autopilot_motors_on = FALSE;
			autopilot_arming_delay_counter = 0;
			if ((radio_control.status == RC_OK) && THROTTLE_STICK_DOWN()) {
				autopilot_arming_state = STATE_MOTORS_OFF_READY;
			} else {
				autopilot_arming_state = STATE_WAITING;
			}
			break;
		case STATE_WAITING:
			//_write(1, "motors waiting ", 15);
			autopilot_motors_on = FALSE;
			autopilot_arming_delay_counter = 0;
			if ((radio_control.status == RC_OK) && THROTTLE_STICK_DOWN()) {
				autopilot_arming_state = STATE_MOTORS_OFF_READY;
			}
			break;
		case STATE_MOTORS_OFF_READY:
			//_write(1, "motors off and ready? ", 22);
			autopilot_motors_on = FALSE;
			autopilot_arming_delay_counter = 0;
			/*if (!THROTTLE_STICK_DOWN()) {
				_write(1, "seats are in the upright position ", 34);
			}
			if (rc_attitude_sticks_centered()) {
				_write(1, "tray tables are closed ", 23);
			}
			if (autopilot_mode == MODE_MANUAL) {
				_write(1, "you are driving ", 16);
			}
			if (autopilot_unarmed_in_auto) {
				_write(1, "what is this anyways? ", 22);
			}*/
			if (!THROTTLE_STICK_DOWN() && rc_attitude_sticks_centered() && (radio_control.status == RC_OK) && (ahrs_aligner.status == AHRS_ALIGNER_LOCKED)) {
				autopilot_arming_state = STATE_ARMING;
			}
			break;
		case STATE_ARMING:
			//_write(1, "arming motors ", 14);
			autopilot_motors_on = FALSE;
			autopilot_arming_delay_counter++;
			if (THROTTLE_STICK_DOWN() || !rc_attitude_sticks_centered() || !(radio_control.status == RC_OK) || !(ahrs_aligner.status == AHRS_ALIGNER_LOCKED)) {
				autopilot_arming_state = STATE_MOTORS_OFF_READY;
			} else if (autopilot_arming_delay_counter >= AUTOPILOT_ARMING_DELAY) {
				autopilot_arming_state = STATE_MOTORS_ON;
			}
			break;
		case STATE_MOTORS_ON:
			//printf("motors armed\n");
			autopilot_motors_on = TRUE;
			autopilot_arming_delay_counter = AUTOPILOT_ARMING_DELAY;
			if (THROTTLE_STICK_DOWN()) {
				autopilot_arming_state = STATE_UNARMING;
			}
			break;
		case STATE_UNARMING:
			//_write(1, "unarming motors ", 16);
			autopilot_motors_on = TRUE;
			autopilot_arming_delay_counter--;
			if (!THROTTLE_STICK_DOWN()) {
				autopilot_arming_state = STATE_MOTORS_ON;
			} else if (autopilot_arming_delay_counter == 0) {
				autopilot_arming_state = STATE_MOTORS_OFF_READY;
			}
			break;
		default:
			break;
		}
}

#define AUTOPILOT_IN_FLIGHT_TIME    40

extern bool_t autopilot_in_flight;
extern uint32_t autopilot_in_flight_counter;
extern uint16_t autopilot_flight_time;

static inline void autopilot_check_in_flight(bool_t motors_on) {
	if (autopilot_in_flight) {
		if (autopilot_in_flight_counter > 0) {
			if (THROTTLE_STICK_DOWN()) {
				autopilot_in_flight_counter--;
				if (autopilot_in_flight_counter == 0) {
					autopilot_in_flight = FALSE;
				}
			} else { /* !THROTTLE_STICK_DOWN */
				autopilot_in_flight_counter = AUTOPILOT_IN_FLIGHT_TIME;
			}
		}
	} else { /* not in flight */
		if (autopilot_in_flight_counter < AUTOPILOT_IN_FLIGHT_TIME
				&& motors_on) {
			if (!THROTTLE_STICK_DOWN()) {
				autopilot_in_flight_counter++;
				if (autopilot_in_flight_counter == AUTOPILOT_IN_FLIGHT_TIME)
					autopilot_in_flight = TRUE;
			} else { /*  THROTTLE_STICK_DOWN */
				autopilot_in_flight_counter = 0;
			}
		}
	}
}

#endif /* AUTOPILOT_ARMING_THROTTLE_H */
